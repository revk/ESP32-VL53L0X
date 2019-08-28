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

typedef enum
{ VcselPeriodPreRange, VcselPeriodFinalRange } vl53l0x_vcselPeriodType;

// Functions returning const char * are OK for NULL, else error string

// Set up I2C and create the vl53l0x structure, NULL means could not see device on I2C
vl53l0x_t *vl53l0x_config (int8_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8);
// Initialise the VL53L0X
const char *vl53l0x_init (vl53l0x_t *);
// End I2C and free the structure
void vl53l0x_end (vl53l0x_t *);

void vl53l0x_setAddress (vl53l0x_t *, uint8_t new_addr);
uint8_t vl53l0x_getAddress (vl53l0x_t * v);

void vl53l0x_writeReg8Bit (vl53l0x_t *, uint8_t reg, uint8_t value);
void vl53l0x_writeReg16Bit (vl53l0x_t *, uint8_t reg, uint16_t value);
void vl53l0x_writeReg32Bit (vl53l0x_t *, uint8_t reg, uint32_t value);
uint8_t vl53l0x_readReg8Bit (vl53l0x_t *, uint8_t reg);
uint16_t vl53l0x_readReg16Bit (vl53l0x_t *, uint8_t reg);
uint32_t vl53l0x_readReg32Bit (vl53l0x_t *, uint8_t reg);

void vl53l0x_writeMulti (vl53l0x_t *, uint8_t reg, uint8_t const *src, uint8_t count);
void vl53l0x_readMulti (vl53l0x_t *, uint8_t reg, uint8_t * dst, uint8_t count);

const char *vl53l0x_setSignalRateLimit (vl53l0x_t *, float limit_Mcps);
float vl53l0x_getSignalRateLimit (vl53l0x_t *);

const char *vl53l0x_setMeasurementTimingBudget (vl53l0x_t *, uint32_t budget_us);
uint32_t vl53l0x_getMeasurementTimingBudget (vl53l0x_t *);

const char *vl53l0x_setVcselPulsePeriod (vl53l0x_t *, vl53l0x_vcselPeriodType type, uint8_t period_pclks);
uint8_t vl53l0x_getVcselPulsePeriod (vl53l0x_t *, vl53l0x_vcselPeriodType type);

void vl53l0x_startContinuous (vl53l0x_t *, uint32_t period_ms);
void vl53l0x_stopContinuous (vl53l0x_t *);
uint16_t vl53l0x_readRangeContinuousMillimeters (vl53l0x_t *);
uint16_t vl53l0x_readRangeSingleMillimeters (vl53l0x_t *);

void vl53l0x_setTimeout (vl53l0x_t *, uint16_t timeout);
uint16_t vl53l0x_getTimeout (vl53l0x_t *);
int vl53l0x_timeoutOccurred (vl53l0x_t *);
int vl53l0x_i2cFail (vl53l0x_t *);


#endif
