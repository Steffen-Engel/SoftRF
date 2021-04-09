/*
 * AHRS.cpp
 *
 *  Created on: 09.04.2021
 *      Author: se
 */
#include "Arduino.h"


#include "AHRS.h"

ahrschip_ops_t *ahrs_chip;

bool  AHRS_probe(void)
{
  return false;
}


byte  AHRS_setup(void)
{
  return 0;
}


void  AHRS_loop(void)
{
}


int AHRS_GDL90(uint8_t *buf)
{
  return 0;
}

void AHRS_NMEA()
{
}

