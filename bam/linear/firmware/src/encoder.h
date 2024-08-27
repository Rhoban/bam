#ifndef _ENCODER_H
#define _ENCODER_H

#include <stdint.h>

void encoder_init();
void encoder_tick();
uint16_t encoder_read();

#endif