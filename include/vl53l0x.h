// VL53L0X control

#ifndef VL53L0X_H
#define VL53L0X_H

typedef struct vl53l0x_s vl53l0x_t;

vl53l0x_t *vl53l0x_init(uint8_t port,uint8_t scl,uint8_t sda,uint8_t address);
void vl532l0x_end(vl53l0x_t *);

#endif
