// VL53L0X control
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>

typedef struct vl53l0x_s vl53l0x_t;

vl53l0x_t *vl53l0x_init(uint8_t port,uint8_t scl,uint8_t sda,uint8_t address);
void vl532l0x_end(vl53l0x_t *);

#endif
