/*
 * CIVARecorder.cpp
 *
 *      Author: Steffen Engel
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "SoC.h"

#include <TimeLib.h>
#include <SdFat.h>
#include <elapsedMillis.h>

#include "../driver/GNSS.h"
#include "../driver/Baro.h"

#include "CIVARecorder.h"
#include "../../SoftRF.h"

#include <SensorQMI8658.hpp>
extern SensorQMI8658 imu_qmi8658;

#define FLIGHTS_DIR "Flights"
extern SdFat uSD;

static File32 LogFile;

static bool inFlight = false;
bool LogActive = false;

void CIVARecorder_setup()
{
  if ((hw_info.storage == STORAGE_CARD)     ||
      (hw_info.storage == STORAGE_FLASH_AND_CARD)
     )
  {
    if (uSD.volumeBegin())
    {
      if (!uSD.exists(FLIGHTS_DIR))
      {
        if (!uSD.mkdir(FLIGHTS_DIR))
        {
           Serial.println(F("Error creating folder on SD!"));
        }
      }
    }
  }

}


void CIVARecorder_loop()
{
#define DetectTakeoffSpeed 50   /* km/h ground speed */
#define DetectLandSpeed 10      /* km/h ground speed */
#define LogInterval 100 /* msec */

  static elapsedMillis TimeNotMoving;
  static elapsedMillis TimeMoving;
  static elapsedMillis LogTime;

  float speed;


  /*****************************************************************************
   * first check if it seems to be flying/moving
   *****************************************************************************/
  // detection by speed
  if (gnss.location.isValid())
  {
    speed = gnss.speed.kmph();
    if (speed >= DetectLandSpeed)
    {
      TimeNotMoving = 0;
    }
    else if (speed < DetectTakeoffSpeed)
    {
      TimeMoving = 0;
    }
  }

  /*****************************************************************************
   * end or start logging, if time of movement change is up
   *****************************************************************************/
  if (LogActive)
  {
    // 15 Seconds not moving? End Logging, close file
    if ((gnss.location.isValid() && (TimeNotMoving >= 15000))
        || (!gnss.location.isValid() && (CIVA_Status == CIVA_LAND)))
    {
      // Close Logfile
      Serial.println("end logging");
      LogFile.close();
      LogActive = false;
    }
  }
  else
  {
    // 10 Seconds moving? Create file
    if (gnss.location.isValid() && (TimeMoving >= 10000)
        || (!gnss.location.isValid() && (CIVA_Status == CIVA_CLIMB)))
    {
      // create Logfile
      Serial.println("start logging");
      char LogName[50];
       snprintf(LogName, sizeof(LogName),
           FLIGHTS_DIR "/%04d-%02d-%02d_%02d-%02d.log",
           year(), month(), day(), hour(), minute());
      LogFile = uSD.open(LogName, FILE_WRITE);
      if (LogFile)
      {
        LogFile.write("date, time, time/msec, nx/g, ny/g, nz/g, phi, psi, theta, rot_x/deg/sec, rot_y/deg/sec, rot_z/deg/s, p_cabin/Pa, p_stat/Pa, p_diff/Pa,eta_hr, eta_qr, eta_sr, eta_fl, GPS_FIX, numSat, GPS_altitude, GPS_speed, longitude, latitude\n");
        LogTime = 0;
        LogActive = true;
      }
    }
  }

  /*****************************************************************************
   * log data to file if logging active
   *****************************************************************************/
  if (LogActive)
  {
    if (LogTime > LogInterval)
    {
      float _nx, _ny, _nz;
      imu_qmi8658.getAccelerometer(_nx, _ny, _nz);
      float _rx, _ry, _rz;
      imu_qmi8658.getGyroscope(_rx, _ry, _rz);


      char loggerline[300];
      // write dataset to logfile
      snprintf(loggerline, sizeof(loggerline), "%02d.%02d.%04d,%02d:%02d:%02d.%d,%8ld,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%6d,%6d,%5d,%4d,%4d,%4d,%4d,%1d,%2d,%5d,%6d,%9d,%9d\n",
                          day(),
                          month(),
                          year(),
                          hour(),
                          minute(),
                          second(),
                          0, // (SensorData.GPS_time/100) % 10,                       //  1
                          millis(),                                                   //  2
                          lround(10*_nx), lround(10*_ny), lround(10*_nz),                                     //  3  4  5
                          0, 0, 0,                                                    //  6  7  8
                          lround(_rx), lround(_ry), lround(_rz),                                              // rotx, y, z //  9 10 11
                          lround(Baro_pressure()), 0, 0,                                      // 12 13 14
                          lround(Baro_altitude() - StartupAltitude), CIVA_Alarm, 0, 0, // SensorData.eta[ETA_HR], SensorData.eta[ETA_QR], SensorData.eta[ETA_SR], SensorData.eta[ETA_FLAP], // 15 16 17 18
                          gnss.location.isValid(), gnss.satellites.value(), lround(gnss.altitude.meters()), lround(gnss.speed.kmph()), // SensorData.GPS_FIX, SensorData.GPS_numSat, SensorData.GPS_altitude, SensorData.GPS_speed,         // 19 20 21 22
                          lround(10000000*gnss.location.lat()), lround(10000000*gnss.location.lng())  // 23 24
                          );
      LogFile.write(loggerline);
      LogTime -= LogInterval;
    }
  }

}


void CIVARecorder_fini()
{
  if (LogActive)
  {
    // close log file and stop logging
    LogFile.close();
    LogActive = false;
  }
}










