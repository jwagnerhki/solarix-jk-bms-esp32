#include <Arduino.h>
#include <stdbool.h>
#include <string.h>

#include "JkBmsData.hpp"

const bool protocol_32 = false;

void decodeBMSData(const byte* raw, JK_BMS_data_t* d)
{
    uint8_t j,i, offset;
    uint32_t runtime;
 
    if (protocol_32 == true) {
        offset = 16;
    } else {
        offset = 0;
    }
    
    if (d==NULL) { return; }

    memset(d, 0x00, sizeof(JK_BMS_data_t));

    if (raw==NULL) { return; }

    // Cell voltages
    for(j=0,i=7; i<NUM_CELLS+24; j++,i+=2) {
        d->cellVoltage[j] = (((int)raw[i] << 8 | raw[i-1])*0.001);
    }

    // +20 byte cells 15 - 24
    // +4 byte  unknown

    d->Average_Cell_Voltage = (((int)raw[59+offset] << 8 | raw[58+offset])*0.001);
    d->Delta_Cell_Voltage = (((int)raw[61+offset] << 8 | raw[60+offset])*0.001);
    d->Current_Balancer = (((int)raw[63+offset] << 8 | raw[62+offset])*0.001);

    // +48 byte cell resistances
    // +6 byte unknown

    if (protocol_32 == true) {
        d->MOS_Temp = (((int)raw[113+2*offset] << 8 | (int)raw[112+2*offset])*0.1);
    }

    d->Battery_Voltage = (((int)raw[121+2*offset] << 24 | raw[120+2*offset] << 16 | raw[119+2*offset] << 8 | raw[118+2*offset])*0.001);

    d->Battery_Power = (((int)raw[125+2*offset] << 24 | (int)raw[124+2*offset] << 16 | (int)raw[123+2*offset] << 8 | (int)raw[122+2*offset])*0.001);
    d->Charge_Current = (((int)raw[129+2*offset] << 24 | raw[128+2*offset] << 16 | raw[127+2*offset] << 8 | raw[126+2*offset])*0.001);
    if (d->Charge_Current < 0) {
        d->Battery_Power = -(d->Battery_Power);
    }

    //Battery_Power = (((int)raw[121] << 8 | raw[120])*0.001); unknown ?!
    //Battery_Power = (((int)raw[123] << 8 | raw[122])*0.001);
    // +6 byte unkown

    //Battery_T1 = (((int)raw[131] << 8 | raw[130])*0.1);
    if(raw[131+2*offset] == 0xFF) {
        d->Battery_T1 = ((0xFF << 24 | 0xFF << 16 | (int)raw[131+2*offset] << 8 | (int)raw[130+2*offset])*0.1);
    } else {
        d->Battery_T1 = (((int)raw[131+2*offset] << 8 | (int)raw[130+2*offset])*0.1);
    }

    //Battery_T2 = (((int)raw[133] << 8 | (int)raw[132])*0.1);
    if(raw[133+2*offset] == 0xFF) {
        d->Battery_T2 = ((0xFF << 24 | 0xFF << 16 | (int)raw[133+2*offset] << 8 | (int)raw[132+2*offset])*0.1);
    } else {
        d->Battery_T2 = (((int)raw[133+2*offset] << 8 | (int)raw[132+2*offset])*0.1);
    }

    //MOS_Temp = (((int)raw[135] << 8 | raw[134])*0.1);
    if (protocol_32 == false) {
        if(raw[135+2*offset] == 0xFF) {
            d->MOS_Temp = ((0xFF << 24 | 0xFF << 16 | (int)raw[135+2*offset] << 8 | (int)raw[134+2*offset])*0.1);
        } else {
            d->MOS_Temp = (((int)raw[135+2*offset] << 8 | (int)raw[134+2*offset])*0.1);
        }
    } else {
        d->alarm_flags = raw[135+2*offset];
        d->charge_over_temp_alarm = raw[135+2*offset] && 0x01;
        d->charge_under_temp_alarm = raw[135+2*offset] && 0x02;
        d->cell_under_voltage_alarm = raw[135+2*offset] && 0x08;
        //Serial.println(charge_under_temp_alarm);
    }

    if (protocol_32 == false) {
        d->alarm_flags = raw[137+2*offset];
        d->charge_over_temp_alarm = raw[137+2*offset] && 0x01;
        d->charge_under_temp_alarm = raw[137+2*offset] && 0x02;
        d->cell_under_voltage_alarm = raw[137+2*offset] && 0x08;
    }

    if((raw[139+2*offset] & 0xF0) == 0x0) {
        d->Balance_Curr = (((int)raw[139+2*offset] << 8 | raw[138+2*offset])*0.001);
    } else if ((raw[139+2*offset] & 0xF0) == 0xF0) {
        d->Balance_Curr = ((((int)raw[139+2*offset] & 0x0F) << 8 | raw[138+2*offset])*-0.001);
    }

    // +2 byte unknown

    d->Percent_Remain = ((int)raw[141+2*offset]);
    d->Capacity_Remain = (((int)raw[145+2*offset] << 24 | raw[144+2*offset] << 16 | raw[143+2*offset] << 8 | raw[142+2*offset])*0.001);
    d->Nominal_Capacity = (((int)raw[149+2*offset] << 24 | raw[148+2*offset] << 16 | raw[147+2*offset] << 8 | raw[146+2*offset])*0.001);
    d->Cycle_Count = (int)raw[153+2*offset] << 24 | (int)raw[152+2*offset] << 16 | (int)raw[151+2*offset] << 8 | (int)raw[150+2*offset];

    // +6 byte unknown

    d->Capacity_Cycle = (((int)raw[161+2*offset] << 8 | raw[160+2*offset])*0.001);
    d->Uptime = (((int)raw[164+2*offset] << 16 | raw[163+2*offset] << 8 | raw[162+2*offset]));
    runtime = d->Uptime;
    
    d->sec = runtime % 60;
    runtime /= 60;
    d->mi = runtime % 60;
    runtime /= 60;
    d->hr = runtime % 24;
    d->days = runtime /= 24;

    // +1 byte unknown

    d->charge = (raw[166+2*offset] > 0);
    d->discharge = (raw[167+2*offset] > 0);

}
