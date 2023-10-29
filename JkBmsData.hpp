#ifndef JK_BMS_DATA_HPP
#define JK_BMS_DATA_HPP

#include <Arduino.h>
#include <stdbool.h>

#define NUM_CELLS 16

typedef struct JK_BMS_data_tt {

    // JK-BMS values
    float cellVoltage[NUM_CELLS+2];
    float Average_Cell_Voltage;
    float Delta_Cell_Voltage;
    float Current_Balancer;
    float Battery_Voltage;
    float Battery_Power;
    float Charge_Current;
    float Battery_T1;
    float Battery_T2;

    float MOS_Temp;
    int Percent_Remain;
    float Capacity_Remain;
    float Nominal_Capacity;
    uint32_t Cycle_Count;
    float Capacity_Cycle;

    uint32_t Uptime;
    uint8_t sec, mi, hr, days;

    float Balance_Curr;
    bool charge;
    bool discharge;

    byte alarm_flags;
    bool charge_over_temp_alarm;
    bool charge_under_temp_alarm;
    bool cell_under_voltage_alarm;

} JK_BMS_data_t;

void decodeBMSData(const byte* pdata, JK_BMS_data_t* d);

#endif
