
//////////////////////////////////////////////////////////////////////////////
// Board: "ESP32 Dev Module"
// Partitioning: Huge App, No OTA
// Serial console 74880 baud
//
// JK-BMS Arduino source code adapted from original at
// https://www.akkudoktor.net/forum/open-source-software-projekte/jkbms-auslesen-ueber-ble-bluetooth-oder-rs485-adapter-mittels-eps-iobroker/paged/42/
//
// Steca Solarix PLI 5000-48 serial rj45 into RS232 / 3.3V LVTTL level
// converter board, and then to RX and TX pins on esp32 board
//    G17 - pin 17 - GPIO17 and UART2 TXD <==> TTL/RS232 board TX in
//    G16 - pin 16 - GPIO16 and UART2 RXD <==> TTL/RS232 board RX out
// DB9 on TTL/rs232 board
//    pin 5 - GND  <==> Steca pin 8   
//    pin 3 - Tx   <==> Steca pin 2 - Rx   
//    pin 2 - Rx   <==> Steca pin 1 - Tx
//
// Zettler AZ850P2-3 set-reset relay
//    G4 - set
//    G5 - reset
//    GND - common ground
//////////////////////////////////////////////////////////////////////////////

#include <BLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

#include "driver/gpio.h"

#include "BistableRelay.hpp"
#include "JkBmsData.hpp"
#include "StecaSolarixData.hpp"

#define SW_Version "1.2"
#define STECA_MAX_CMD_LEN      50
#define STECA_MAX_RESPONSE_LEN 200

#define GPIO_SET_ZETTLER_RELAY   4  // GPIO #4
#define GPIO_RESET_ZETTLER_RELAY 5  // GPIO #5

const bool debug_flg = true;
const bool debug_ble_callback = false;

//////////////////////////////////////////////////////////////////////////////

const bool MQTT_ENABLE = true;
const String mqttname = "solar";

const int mqttpublishtime_period = 5000;  // millisec
const int solarixquerytime_interval = 5000; // millisec

const char mqtt_server[] = "192.168.0.74";
const int mqtt_port = 1883;
const char* mqtt_username = "";
const char* mqtt_passwort = "";

const char ssid[] = "---add own---";
const char password[] = "---add own---";

const char* JK_BMS_NAME = "JK-B2A24S15P";

const int mqttpublishtime_interval = 5000; // millisec

enum receiveSolarixResponse_t { SOLARIX_RESPONSE_MORE=0, SOLARIX_RESPONSE_OK, SOLARIX_RESPONSE_CRC_ERROR, SOLARIX_RESPONSE_OVERFLOW_ERROR };
uint16_t calcCrc16(const uint8_t* data, size_t nbytes);
void sendSolarixCommand(const char* cmd, size_t cmdlen);
receiveSolarixResponse_t receiveSolarixResponse(char* response, size_t* responselen);

//////////////////////////////////////////////////////////////////////////////

void mqttCallback(char* topic, byte* payload, unsigned int length);
void decodeBMSData(const byte* pdata, const bool dump_data);

WiFiClient espClient;
PubSubClient mqttclient(espClient);
WebServer httpserver(80);

BLEClient* pBLEClient;
BLEScan* pBLEScan;
BLERemoteCharacteristic* pRemoteCharacteristic;
BLEAdvertisedDevice* myDevice;

HardwareSerial SolarixSerial(2); // UART2

BistableRelay stecaPoweronRelay(GPIO_SET_ZETTLER_RELAY, GPIO_RESET_ZETTLER_RELAY); // controls Solarix On/Off switch

//////////////////////////////////////////////////////////////////////////////

#define willMessage "offline"
String willTopic = mqttname + "/status";
byte willQoS = 0;
boolean willRetain = true;

long lastReconnectAttempt = 0;

static BLEUUID serviceUUID("ffe0"); // The remote service we wish to connect to.
static BLEUUID    charUUID("ffe1"); //ffe1 // The characteristic of the remote service we are interested in.
byte getdeviceInfo[20] = {0xaa, 0x55, 0x90, 0xeb, 0x97, 0x00, 0xdf, 0x52, 0x88, 0x67, 0x9d, 0x0a, 0x09, 0x6b, 0x9a, 0xf6, 0x70, 0x9a, 0x17, 0xfd}; // Device Infos
//byte getInfo[20] = {0xaa, 0x55, 0x90, 0xeb, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
byte getInfo[20] = {0xaa, 0x55, 0x90, 0xeb, 0x96, 0x00, 0x79, 0x62, 0x96, 0xed, 0xe3, 0xd0, 0x82, 0xa1, 0x9b, 0x5b, 0x3c, 0x9c, 0x4b, 0x5d};
//byte balancer_an[20] = {0xaa, 0x55, 0x90, 0xeb, 0x1f, 0x04, 0x01, 0x00, 0x00, 0x00, 0xd3, 0x27, 0x86, 0x3b, 0xe2, 0x12, 0x4d, 0xb0, 0xb6, 0x00};

unsigned long sendingtime = 0;
unsigned long bleScantime = 0;
unsigned long mqttpublishtime = 0;
unsigned long newdatalasttime = 0;
unsigned long ble_connection_time = 0;
unsigned long solarixquerytime = 0;

const int num_bytes = 320;
byte receivedBytes_main[num_bytes];

int frame = 0;
bool received_start = false;
bool received_start_frame = false;
bool received_complete = false;
bool new_data = false;
byte BLE_Scan_counter = 0;

// JK-BMS values
JK_BMS_data_t jk;

// Solarix values
Steca_Solarix_data_t solarix;

// WWW
String HTML = "Starting";

static bool doConnect = false;
static bool ble_connected = false;

//////////////////////////////////////////////////////////////////////////////

void initWiFi()
{
    byte wifi_retry = 0;
    WiFi.disconnect();
    WiFi.persistent(false); // do not persist as WiFi.begin is not helpful then. Decreases connection speed but helps in other ways
    WiFi.mode(WIFI_STA);
    //esp_wifi_set_ps( WIFI_PS_NONE ); // do not set WiFi to sleep, increases stability
    WiFi.config(0u, 0u, 0u); // ensure settings are reset, retrieve new IP Address in any case => DHCP necessary

    WiFi.begin(ssid, password);
    Serial.printf("Connecting to WiFi '%s' ...", ssid);

    while (WiFi.status() != WL_CONNECTED && wifi_retry < 10  ) {
        WiFi.begin(ssid, password);
        wifi_retry++;
        Serial.print('.');
        delay(10000);
    }
    if(wifi_retry >= 10) {
        Serial.println(F("\nReboot..."));
        ESP.restart();
    }

    Serial.print(F("Ready. IP addr: "));
    Serial.println(WiFi.localIP());
}


class MyClientCallback : public BLEClientCallbacks {

    void onConnect(BLEClient* pclient) { }

    void onDisconnect(BLEClient* pclient)
    {
        ble_connected = false;
        //pclient->disconnect();
        Serial.println(F("BLE-Disconnect"));
        String topic = mqttname + "/BLEconnection";
        mqttclient.publish(topic.c_str(), "disconnected & Rebooting");
        delay(200);
        mqttclient.loop();
        delay(200);
        Serial.println(F("BLE was Disconnected, no reconnection possible, Reboot ESP..."));
        ESP.restart();
    }

};


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
    * Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print(F("BLE Advertised Device found: "));
        Serial.println(advertisedDevice.toString().c_str());

        // We have found a device, let us now see if it contains the service we are looking for.
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID) && advertisedDevice.getName() == JK_BMS_NAME) {
          pBLEScan->stop();
          myDevice = new BLEAdvertisedDevice(advertisedDevice);
          doConnect = true;
          pBLEScan->clearResults();
        }
    }
};

bool connectToBLEServer()
{
    Serial.print(F("BLE Connecting to "));
    Serial.println(myDevice->getAddress().toString().c_str());

    pBLEClient->setClientCallbacks(new MyClientCallback());
    delay(500); // hope it helps against ->  lld_pdu_get_tx_flush_nb HCI packet count mismatch (0, 1)

    // Connect to the remove BLE Server.
    pBLEClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    delay(500); // hope it helps against ->  lld_pdu_get_tx_flush_nb HCI packet count mismatch (0, 1)
    Serial.println(F(" - Connected"));

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pBLEClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.print(F("Failed to find service UUID: "));
        Serial.println(serviceUUID.toString().c_str());
        pBLEClient->disconnect();
        return false;
    }
    Serial.println(F(" - Found service"));

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
        Serial.print(F("Failed to find characteristic UUID: "));
        Serial.println(charUUID.toString().c_str());
        pBLEClient->disconnect();
        return false;
    }
    Serial.println(F(" - Found characteristic"));

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
        std::string value = pRemoteCharacteristic->readValue();
        Serial.print(F("The characteristic value was: "));
        Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        Serial.println(F("Notify the characteristic"));
    }

    // Sending getdevice info
    pRemoteCharacteristic->writeValue(getdeviceInfo, sizeof(getdeviceInfo));
    sendingtime = millis();
    Serial.println(F("Sending device Info"));

    ble_connected = true;
    doConnect = false;
    Serial.println(F("Connected to the BLE Server."));
    String topic = mqttname + "/BLEconnection";
    mqttclient.publish(topic.c_str(),"connected");

    return true;
}

//////////////////////////////////////////////////////////////////////////////

static uint8_t steca_rs232_response_buf[STECA_MAX_RESPONSE_LEN] = { '\0' };
static size_t steca_rs232_response_buf_len = 0;

// Calculate a 16-bit checksum using the CRC-CCITT (XMODEM) algorithm,
// copied from https://github.com/particle-iot/esp32-ncp-firmware/blob/master/main/xmodem_receiver.cpp
uint16_t calcCrc16(const uint8_t* data, size_t nbytes)
{
    uint16_t crc = 0;
    const auto end = data + nbytes;
    while (data < end) {
        const uint8_t c = *data++;
        crc ^= (uint16_t)c << 8;
        for (unsigned i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


void sendSolarixCommand(const char* cmd, size_t cmdlen)
{
    uint16_t crc16 = calcCrc16((const uint8_t*)cmd, cmdlen);
    SolarixSerial.write(cmd, cmdlen);
    SolarixSerial.write((byte)(crc16 >> 8));
    SolarixSerial.write((byte)(crc16 & 0xFF));
    SolarixSerial.write('\r');
    Serial.write(cmd, cmdlen);
    Serial.printf("<crc16:0x%04x>", crc16);
    Serial.println("\\r");
}

receiveSolarixResponse_t receiveSolarixResponse(char* response, size_t* responselen)
{
    static bool synced = false;
    
    *response = '\0';
    *responselen = 0;
    
    while(SolarixSerial.available()) {
  
      uint8_t ch = SolarixSerial.read();
  
      if (ch == '(' && steca_rs232_response_buf_len == 0) {
        synced = true;
      }
  
      //Serial.printf("rx: %c  synced: %d   buflen: %d\n", ch, synced, (int)steca_rs232_response_buf_len);
  
      if (!synced) {
        steca_rs232_response_buf_len = 0;
        continue;
      }
  
      if (ch == '\r') {
  
        *responselen = steca_rs232_response_buf_len - 3; // discard the starting '(' and trailing <crc16>
        for (size_t i = 0; i < steca_rs232_response_buf_len-3; i++) {
          response[i] = steca_rs232_response_buf[i+1];
        }
        
        uint16_t crc16 = steca_rs232_response_buf[steca_rs232_response_buf_len-2];
        crc16 = (crc16 << 8) + steca_rs232_response_buf[steca_rs232_response_buf_len-1];
        
        uint16_t own_crc16 = calcCrc16(steca_rs232_response_buf, steca_rs232_response_buf_len - 2);
        //uint16_t own_crc16 = calcCrc16(steca_rs232_response_buf+1, steca_rs232_response_buf_len - 3); // without prefixed '('
  
        synced = false;
        steca_rs232_response_buf_len = 0;
  
        if (crc16 != own_crc16) {
          //Serial.printf("<crc16 remote: 0x%04x> ", crc16);
          //Serial.printf("<crc16 local: 0x%04x>\n", own_crc16);
          return SOLARIX_RESPONSE_CRC_ERROR;
        }
        return SOLARIX_RESPONSE_OK;
  
      } else {
        
        steca_rs232_response_buf[steca_rs232_response_buf_len] = ch;
        steca_rs232_response_buf_len++;
        if (steca_rs232_response_buf_len >= sizeof(steca_rs232_response_buf)) {
          synced = false;
          steca_rs232_response_buf_len = 0;
          return SOLARIX_RESPONSE_OVERFLOW_ERROR;
        } 
      }
  
    }
    
    return SOLARIX_RESPONSE_MORE;
}

//////////////////////////////////////////////////////////////////////////////

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    String Command = "";

    if(strcmp(topic, "inverter/enable") == 0) {
        for (int i = 0; i < length; i++) {
            Command = Command + (char)payload[i];
        }
        if(Command == "true") {
            Serial.println("Enable inverter: true (on)");
            stecaPoweronRelay.set();
        } else if(Command == "false") {
            Serial.println("Enable inverter: false (off)");
            stecaPoweronRelay.reset();
        }
    }
}

boolean mqttReconnect()
{
    String topic2 = mqttname + "/BLEconnection";
    if (mqttclient.connect(mqttname.c_str(), mqtt_username, mqtt_passwort, willTopic.c_str(), willQoS, willRetain, willMessage)) {
        // Once connected, publish an announcement...
        if(millis() < 20000) {
          mqttclient.publish(topic2.c_str(),"Startup");
        }

        topic2 = mqttname + "/inverter/enable";
        mqttclient.subscribe(topic2.c_str());

        topic2 = mqttname + "/status";
        mqttclient.publish(topic2.c_str(),"online");
        if(debug_flg) {
          Serial.println("MQTT reconnected!");
        }
    } else {
        if(debug_flg) {
            Serial.println("MQTT connection failed!");
        }
    }

    return mqttclient.connected();
}

// Build a JSON string of values and also send out MQTT topics
void mqttPublish()
{
    String cellStr;
    String topic;
    String cellVoltageBaseTopic = mqttname + "/data/cell_";

    HTML= "{\"Cell\":{";
    for(uint8_t i=0; i<NUM_CELLS; i++) {
        cellStr = String(jk.cellVoltage[i],3);
        if(i<9) topic = cellVoltageBaseTopic + String("0") + String(i+1);
        else topic = cellVoltageBaseTopic + String(i+1);
        if(jk.cellVoltage[i] != 0) {
            if (MQTT_ENABLE) {
              mqttclient.publish(topic.c_str(), cellStr.c_str());
            }
          HTML = HTML + "\""  + String(i) + "\":" + cellStr.c_str();
          if((i < NUM_CELLS-1) && (jk.cellVoltage[i+1] != 0)) {
              HTML = HTML + ",";
          }
        }
    }

    HTML = HTML + "}," + "\"Battery\":{";
    cellStr = String(jk.Battery_Voltage,3);
    topic = mqttname + "/data/Battery_Voltage";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Battery_Voltage\":" + cellStr.c_str() + ",";

    cellStr = String(jk.Delta_Cell_Voltage,3);
    topic = mqttname + "/data/Delta_Cell_Voltage";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Delta_Cell_Voltage\":" + cellStr.c_str() + ",";

    cellStr = String(jk.MOS_Temp,3);
    topic = mqttname + "/data/MOS_Temp";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"MOS_Temp\":" + cellStr.c_str() + ",";

    cellStr = String(jk.Battery_T1,3);
    topic = mqttname + "/data/Battery_T1";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Battery_T1\":" + cellStr.c_str() + ",";

    cellStr = String(jk.Battery_T2,3);
    topic = mqttname + "/data/Battery_T2";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Battery_T2\":" + cellStr.c_str() + ",";

    cellStr = String(jk.Battery_Power,3);
    topic = mqttname + "/data/Battery_Power";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Battery_Power\":" + cellStr.c_str() + ",";

    cellStr = String(jk.Charge_Current,3);
    topic = mqttname + "/data/Charge_Current";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Charge_Current\":" + cellStr.c_str() + ",";

    cellStr = String(jk.Percent_Remain);
    topic = mqttname + "/data/Percent_Remain";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Percent_Remain\":" + cellStr.c_str() + ",";

    cellStr = String(jk.Cycle_Count);
    topic = mqttname + "/data/Cycle_Count";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Cycle_Count\":" + cellStr.c_str() + ",";

    // Capacity_Remain
    // Nominal_Capacity
    // Capacity_Cycle

    cellStr = String(jk.Balance_Curr,3);
    topic = mqttname + "/data/Balance_Current";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Balance_Current\":" + cellStr.c_str() + ",";

    cellStr = String(jk.sec);
    topic = mqttname + "/uptime/secs";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Uptime_Sec\":" + cellStr.c_str() + ",";

    cellStr = String(jk.mi);
    topic = mqttname + "/uptime/mins";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Uptime_Min\":" + cellStr.c_str() + ",";

    cellStr = String(jk.hr);
    topic = mqttname + "/uptime/hours";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Uptime_Hr\":" + cellStr.c_str() + ",";

    cellStr = String(jk.days);
    topic = mqttname + "/uptime/days";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Uptime_Day\":" + cellStr.c_str() + ",";

    cellStr = String(jk.charge);
    topic = mqttname + "/data/Charge";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    cellStr = String(jk.discharge);
    HTML = HTML + "\"Charge\":\"" + cellStr.c_str() + "\",";
    topic = mqttname + "/data/Discharge";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Discharge\":\"" + cellStr.c_str() + "\",";

    HTML += "}";
    HTML += "\"Steca\":{";

    cellStr = String(solarix.load_S_VA);
    topic = mqttname + "/data/Steca_Load_VA";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Steca_Load_VA\":" + cellStr.c_str() + ",";

    cellStr = String(solarix.load_P_Watt);
    topic = mqttname + "/data/Steca_Load_W";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Steca_Load_W\":" + cellStr.c_str() + ",";

    cellStr = String(solarix.pv_P_Watt);
    topic = mqttname + "/data/Steca_MPPT_W";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Steca_MPPT_W\":" + cellStr.c_str() + ",";

    cellStr = String(solarix.pv_V);
    topic = mqttname + "/data/Steca_MPPT_V";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Steca_MPPT_V\":" + cellStr.c_str() + ",";

    cellStr = String(solarix.pv_I);
    topic = mqttname + "/data/Steca_MPPT_I";
    if (MQTT_ENABLE) {
        mqttclient.publish(topic.c_str(),cellStr.c_str());
    }
    HTML = HTML + "\"Steca_MPPT_I\":" + cellStr.c_str() + "}";

    HTML += "}";
}


static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
    if(debug_ble_callback)
    {
        Serial.print("Notify callback for characteristic ");
        Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
        Serial.print(" of data length ");
        Serial.println(length);
        Serial.print("data: ");
        for (int i = 0; i < length; i++)  {
          Serial.print(pData[i],HEX);
          Serial.print(", ");
        }
        Serial.println("");
    }

    if(pData[0] == 0x55 && pData[1] == 0xAA && pData[2] == 0xEB && pData[3] == 0x90 && pData[4] == 0x02) {
      if(debug_ble_callback) { Serial.println("Daten anerkannt !"); }
      int newlen = length;
      if(newlen > sizeof(receivedBytes_main)) { newlen = num_bytes; } //Prevents writing outside the array
      received_start = true;
      received_start_frame = true;
      received_complete = false;
      frame = 0;
      for (int i = 0; i < newlen; i++)  {
       receivedBytes_main[frame] = pData[i];
       frame++;
      }
    }

    if(received_start && !received_start_frame && !received_complete) {
      if(debug_ble_callback) { Serial.println("Daten erweitert !"); }
      int newlenadd=300-(frame+length); //Prevents writing outside the array
      if(newlenadd>=0){newlenadd=length;} //Prevents writing outside the array
      if(newlenadd<0){newlenadd=300-frame;} //Prevents writing outside the array
      for (int i = 0; i < newlenadd; i++)  {
        receivedBytes_main[frame] = pData[i];
        frame++;
      }

      if(frame == 300) {
        if(debug_ble_callback) { Serial.println("New Data for Analyse Complete..."); }
        received_complete = true;
        received_start = false;
        new_data = true;
        BLE_Scan_counter = 0;
      } else if((frame > 300)) {
        Serial.println("Fehlerhafte BMS/Bluetooth Daten !!");
        frame = 0;
        received_start = false;
        new_data = false;
      }

    }
    //Serial.print("frame: ");
    //Serial.println(frame);
    received_start_frame = false;
}


// Handle root url (/)
void handle_root()
{
  httpserver.send(200, "application/json", HTML);
}

void setup()
{
    SolarixSerial.begin(2400, SERIAL_8N1, 16, 17);
    steca_rs232_response_buf_len = 0;

    Serial.begin(74880);
    Serial.print("BMS Publisher V ");
    Serial.println(SW_Version);
    Serial.println("Booting");

    // Init relay object
    stecaPoweronRelay.setup();

    //WIFI Setup
    initWiFi();

    // WebServer
    httpserver.on("/", handle_root);
    httpserver.begin();
    Serial.println("Webserver gestartet");

    // MQTT
    mqttclient.setServer(mqtt_server, mqtt_port);
    mqttclient.setCallback(mqttCallback);

    //BLE Setup
    BLEDevice::init("");
    pBLEClient = BLEDevice::createClient();
    Serial.println(" - Created client");
  
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
}

void loop()
{
    static char steca_response[STECA_MAX_RESPONSE_LEN] = { '\0' };
    static size_t steca_response_len = 0;

    // Relay control
    stecaPoweronRelay.loop();

    // WiFi (Re)Connect
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WIFI Connection is Lost! Try to Reconnect..."));
        initWiFi();
    } else {
        if (MQTT_ENABLE) {
            if (!mqttclient.connected()) {
                if ((millis() - lastReconnectAttempt) > 5000) { // reconnect nach 5 sekunden
                    if(debug_flg) {
                        Serial.println("MQTT Client not connected");
                        Serial.println("MQTT time for reconnect");
                    }
                    lastReconnectAttempt = millis();
                    // Attempt to reconnect
                    if (mqttReconnect()) {
                        lastReconnectAttempt = 0;
                    } else {
                        if(debug_flg) {
                            Serial.println("MQTT reconnect Error");
                        }
                    }
                }
            } else {
              mqttclient.loop();
            }
        }
    }

    // Bluetooth (re)connect
    if (doConnect == true) {
        if(!connectToBLEServer()) {
            Serial.println(F("We have failed to connect to the server; there is nothin more we will do."));
            String topic = mqttname + "/BLEconnection";
            if (MQTT_ENABLE) {
                mqttclient.publish(topic.c_str(),"BLE_Connection_error!");
            }
            delay(500);
            ble_connected = false;
            doConnect = false;
        }
    }

    // JK-BMS Response Processing
    if (ble_connected) {
        if(received_complete) {
            //    for (int i = 0; i < 319; i++)  {
            //      Serial.print(receivedBytes_main[i],HEX);
            //     Serial.print(", ");
        }
        
        bool do_publish_data = (mqttpublishtime == 0 || (millis() >= (mqttpublishtime + mqttpublishtime_interval)));
        
        if(new_data) {
            decodeBMSData(receivedBytes_main, &jk);
            newdatalasttime = millis();
            new_data = false;
        }

        if(do_publish_data) {
            mqttpublishtime = millis();
            mqttPublish();

            String topic = mqttname + "/BLEconnection";
            if (MQTT_ENABLE) {
                mqttclient.publish(topic.c_str(),"connected");
                topic = mqttname + "/status";
                mqttclient.publish(topic.c_str(),"online");
            }
        }
    }

    // JK-BMS Query Dispatch
    if(((millis() - sendingtime) > 500) && sendingtime != 0) {
        sendingtime = 0;
        Serial.println("gesendet!");
        pRemoteCharacteristic->writeValue(getInfo, sizeof(getInfo));
    }


    // Bluetooth timeout / reconnect
    if((!ble_connected && !doConnect && (millis() - bleScantime) > 15000)) {
        Serial.println("BLE -> Reconnecting " + String(BLE_Scan_counter));
        String topic = mqttname + "/BLEconnection";
        String msg = "Reconnecting_" + String(BLE_Scan_counter);
        mqttclient.publish(topic.c_str(),msg.c_str());
        bleScantime = millis();
        pBLEScan->start(5, false);
        BLE_Scan_counter++;
    }

    // Bluetooth connected but no data since a while
    if(!doConnect && ble_connected && (millis() >= (newdatalasttime + 60000)) && newdatalasttime != 0){
        ble_connected = false;
        delay(200);
        String topic = mqttname + "/BLEconnection";
        mqttclient.publish(topic.c_str(),"terminated");
        Serial.println("BLE-Disconnect/terminated");
        newdatalasttime = millis();
        pBLEClient->disconnect();
    }

    //checker das nach max 5 Minuten und keiner BLE Verbidung neu gestartet wird...
    if(BLE_Scan_counter > 20) {
        String topic = mqttname + "/BLEconnection";
        mqttclient.publish(topic.c_str(),"Rebooting");
        delay(200);
        mqttclient.loop();
        delay(200);
        Serial.println(F("BLE isn't receiving new Data form BMS... and no BLE reconnection possible, Reboot ESP..."));
        ESP.restart();
    }

    // Solarix Status Query Dispatch
    if(solarixquerytime == 0 || (millis() >= (solarixquerytime + solarixquerytime_interval))) {
        solarixquerytime = millis();
        sendSolarixCommand("QPIGS", 5);
    }

    // Solarix Response
    receiveSolarixResponse_t rc = receiveSolarixResponse(steca_response, &steca_response_len);
    switch(rc) {
      case SOLARIX_RESPONSE_MORE:
        break;
      case SOLARIX_RESPONSE_OVERFLOW_ERROR:
        Serial.println("Error: Steca response exceeds hard-coded buffer size");
        break;
      case SOLARIX_RESPONSE_CRC_ERROR:
        Serial.println("Error: Checksum error in Steca response");
        //break;  // or, fall through to:
      case SOLARIX_RESPONSE_OK:
        Serial.print("Response: [");
        Serial.write(steca_response, steca_response_len);
        Serial.println("]");

        decodeStecaSolarixData(steca_response, &solarix);
        // 00:36:00.163 -> Response: [000.0 00.0 230.0 50.0 0069 0020 001 373 53.00 000 053 0025 0000 000.0 00.00 00000 00010000 00 00 00000 010]
        //                            acV   fq   ac_out fq ac_VA Watt bat% busV batV btI cap tempC pvI pvV  batV  batIdisc stat  qq vv  pvW  stat
        solarix.load_S_VA = (steca_response[22]-'0')*1000 + (steca_response[23]-'0')*100 + (steca_response[24]-'0')*10 + (steca_response[25]-'0');
        solarix.load_P_Watt = (steca_response[27]-'0')*1000 + (steca_response[28]-'0')*100 + (steca_response[29]-'0')*10 + (steca_response[30]-'0');
        solarix.pv_P_Watt = (steca_response[97]-'0')*10000 + (steca_response[98]-'0')*1000 + (steca_response[99]-'0')*100 + (steca_response[100]-'0')*10 + (steca_response[101]-'0');
        solarix.pv_V = (steca_response[64]-'0')*100 + (steca_response[65]-'0')*10 + (steca_response[66]-'0');
        solarix.pv_I = (steca_response[59]-'0')*1000 + (steca_response[60]-'0')*100 + (steca_response[61]-'0')*10 + (steca_response[62]-'0');
        break;
      default:
        break;
    }
    
    httpserver.handleClient();

    stecaPoweronRelay.loop();
}
// end loop
