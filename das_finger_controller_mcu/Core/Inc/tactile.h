#ifndef _TACTILE_H
#define _TACTILE_H

#include <stdint.h>

extern uint16_t crc_tactile_R;
extern uint16_t crc_tactile_L;
uint8_t tactile_IsCRCOK(const uint8_t *data, uint16_t length);
void tactile_deal_with(uint8_t *Txdata,const uint8_t *Rxdata);

#endif
