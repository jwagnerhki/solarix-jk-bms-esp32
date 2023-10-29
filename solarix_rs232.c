
#include "solarix_rs232.h"

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

// Decode response to QPIGS command.
void decodeSolarix_QPIGS(const char* steca_response)
{
	steca_load_S_VA = (steca_response[22]-'0')*1000 + (steca_response[23]-'0')*100 + (steca_response[24]-'0')*10 + (steca_response[25]-'0');
	steca_load_P_Watt = (steca_response[27]-'0')*1000 + (steca_response[28]-'0')*100 + (steca_response[29]-'0')*10 + (steca_response[30]-'0');
	steca_pv_P_Watt = (steca_response[97]-'0')*10000 + (steca_response[98]-'0')*1000 + (steca_response[99]-'0')*100 + (steca_response[100]-'0')*10 + (steca_response[101]-'0');
	steca_pv_V = (steca_response[64]-'0')*100 + (steca_response[65]-'0')*10 + (steca_response[66]-'0');
	steca_pv_I = (steca_response[59]-'0')*1000 + (steca_response[60]-'0')*100 + (steca_response[61]-'0')*10 + (steca_response[62]-'0');
]
