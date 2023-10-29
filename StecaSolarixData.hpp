#ifndef STECA_SOLARIX_DATA_HPP
#define STECA_SOLARIX_DATA_HPP

#include <Arduino.h>
#include <stdbool.h>

typedef struct Steca_Solarix_data_tt {

    uint16_t load_S_VA;
    uint16_t load_P_Watt;
    uint16_t pv_P_Watt;
    uint16_t pv_V;
    uint16_t pv_I;

} Steca_Solarix_data_t;

void decodeStecaSolarixData(const char* raw, Steca_Solarix_data_t* d);

#endif
