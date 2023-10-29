#include <Arduino.h>
#include <stdbool.h>
#include <string.h>

#include "StecaSolarixData.hpp"

const bool protocol_32 = false;

void decodeStecaSolarixData(const char* raw, Steca_Solarix_data_t* d)
{
	// 00:36:00.163 -> Response: [000.0 00.0 230.0 50.0 0069 0020 001 373 53.00 000 053 0025 0000 000.0 00.00 00000 00010000 00 00 00000 010]
	//                            acV   fq   ac_out fq ac_VA Watt bat% busV batV btI cap tempC pvI pvV  batV  batIdisc stat  qq vv  pvW  stat
	d->load_S_VA = (raw[22]-'0')*1000 + (raw[23]-'0')*100 + (raw[24]-'0')*10 + (raw[25]-'0');
	d->load_P_Watt = (raw[27]-'0')*1000 + (raw[28]-'0')*100 + (raw[29]-'0')*10 + (raw[30]-'0');
	d->pv_P_Watt = (raw[97]-'0')*10000 + (raw[98]-'0')*1000 + (raw[99]-'0')*100 + (raw[100]-'0')*10 + (raw[101]-'0');
	d->pv_V = (raw[64]-'0')*100 + (raw[65]-'0')*10 + (raw[66]-'0');
	d->pv_I = (raw[59]-'0')*1000 + (raw[60]-'0')*100 + (raw[61]-'0')*10 + (raw[62]-'0');
}
