/*
 * AHRS.h
 *
 *  Created on: 09.04.2021
 *      Author: se
 */

#ifndef SRC_DRIVER_AHRS_H_
#define SRC_DRIVER_AHRS_H_


#define AHRS_INTERVAL (1000/20)

enum
{
  AHRS_MODULE_NONE,
  AHRS_MODULE_MPU9250,
  AHRS_MODULE_BNO55
};

typedef struct ahrschip_ops_struct {
  byte type;
  const char name[10];
  bool (*probe)();
  void (*setup)();
  float (*altitude)(float);
  float (*pressure)();
  float (*temperature)();
} ahrschip_ops_t;

extern ahrschip_ops_t *ahrs_chip;

bool  AHRS_probe(void);
byte  AHRS_setup(void);
void  AHRS_loop(void);

void AHRS_NMEA();
int AHRS_GDL90(uint8_t *buf);

#endif /* SRC_DRIVER_AHRS_H_ */
