#ifndef GUARD_ENDIAN_H
#define GUARD_ENDIAN_H
//
//  endian.h
//  HRD_Serial_Server_C
//
//  Created by Sivon Toledo.
//  Modified by Steve Baker.
//
#include <stdint.h>

uint32_t uint32FromLittleEndian(uint8_t *buffer);
void uint32ToLittleEndian(uint32_t x, uint8_t *buffer);
uint16_t uint16FromLittleEndian(uint8_t *buffer);
void uint16ToLittleEndian(uint16_t x, uint8_t *buffer);

#endif
