#ifndef SOLARIX_RS232_H
#define SOLARIX_RS232_H

#define STECA_MAX_CMD_LEN      50
#define STECA_MAX_RESPONSE_LEN 200

enum receiveSolarixResponse_t { SOLARIX_RESPONSE_MORE=0, SOLARIX_RESPONSE_OK, SOLARIX_RESPONSE_CRC_ERROR, SOLARIX_RESPONSE_OVERFLOW_ERROR };

uint16_t calcCrc16(const uint8_t* data, size_t nbytes);

void sendSolarixCommand(const char* cmd, size_t cmdlen);

receiveSolarixResponse_t receiveSolarixResponse(char* response, size_t* responselen);

#endif
