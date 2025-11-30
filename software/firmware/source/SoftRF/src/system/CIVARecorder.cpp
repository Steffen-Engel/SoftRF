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

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
void CIVARecorder_setup()   {}
void CIVARecorder_loop()    {}
void CIVARecorder_fini()    {}
bool LogActive = false;
#else

#include <TimeLib.h>
#include <SdFat.h>
#include <elapsedMillis.h>

#include "../driver/GNSS.h"
#include "../driver/Baro.h"

#include "CIVARecorder.h"
#include "../../SoftRF.h"

#include <SensorQMI8658.hpp>
extern SensorQMI8658 imu_qmi8658;
#include <SensorQMC6310.hpp>
extern SensorQMC6310 mag_qmc6310;

#define FLIGHTS_DIR "Flights"
extern SdFat uSD;

static File32 LogFile;

bool LogActive = false;


void ShortBeep()
{
//  digitalWrite(SOC_GPIO_PIN_CIVA_BUZZER, HIGH);
//  delay(100);
//  digitalWrite(SOC_GPIO_PIN_CIVA_BUZZER, LOW);
}

void CIVARecorder_setup()
{

  delay(500);
  ShortBeep();
  delay(50);
  ShortBeep();


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

#define USEGPS 1

#if USEGPS
#define DetectTakeoffSpeed 50   /* km/h ground speed */
#define DetectLandSpeed 10      /* km/h ground speed */

  static elapsedMillis TimeNotMoving;
  static elapsedMillis TimeMoving;
  int32_t speed;

#endif

#define LogInterval 100 /* msec */
  static elapsedMillis LogTime;
  /*****************************************************************************
   * first check if it seems to be flying/moving
   *****************************************************************************/
  // detection by speed
#if USEGPS
  if (isValidGNSSFix())
  {
    speed = lround(gnss.speed.kmph());

    // if faster than take off detection, the not moving time has to be zeroed
    if (speed >= DetectLandSpeed)
    {
      TimeNotMoving = 0;
    }

    // if slower than landing detection, the moving time has to be zeroed
    if (speed < DetectTakeoffSpeed)
    {
      TimeMoving = 0;
    }
  }
  else
  {
    if ((CIVA_Status == CIVA_GROUND) || (CIVA_Status == CIVA_LAND))
    {
      TimeMoving = 0;
    }
    else
    {
      TimeNotMoving = 0;
    }
  }
#endif

  /*****************************************************************************
   * end or start logging, if time of movement change is up
   *****************************************************************************/
  if (LogActive)
  {
#if USEGPS
    // 15 Seconds not moving? End Logging, close file
    if (TimeNotMoving >= 15000)
#else
      if ((CIVA_Status == CIVA_GROUND)
          || (CIVA_Status == CIVA_LAND)
         )
#endif
      {
      // Close Logfile
      Serial.println("end logging");

      ShortBeep();
      delay(50);
      ShortBeep();

      char LogText[100];
           snprintf(LogText, sizeof(LogText),
               " Log Ended at %d msecs with GPS speed %d Sats %d NotMoving %d moving %d CIVA Status %d\n",
               millis(), lround(gnss.speed.kmph()), gnss.satellites.value(), (int)TimeNotMoving, (int)TimeMoving, CIVA_Status);

      Serial.print(LogText);
      LogFile.write(LogText);
      LogFile.close();
      LogActive = false;
    }
  }
  else
  {
#if USEGPS
    // 2 Seconds moving? Create file
    if (TimeMoving >= 2000)
#else
      if ((CIVA_Status == CIVA_CLIMB)         // should be they way of activation
          || (CIVA_Status == CIVA_ALT150)     // be sure if climb missed
          || (CIVA_Status == CIVA_ALT200)     // be sure if climb missed
          || (CIVA_Status == CIVA_ACTIVE)     // be sure if climb missed
          )
#endif
      {
      // create Logfile
      Serial.println("start logging");
      ShortBeep();

      char LogName[50];
       snprintf(LogName, sizeof(LogName),
           FLIGHTS_DIR "/%04d-%02d-%02d_%02d-%02d.dat",
           year(), month(), day(), hour(), minute());
      LogFile = uSD.open(LogName, O_WRONLY | O_CREAT);
      if (LogFile)
      {

        LogFile.write(
            "      date,    time, time/msec,"                        //  1  2  3
            " nx/g, ny/g, nz/g,"                                               //  4  5  6
            " phi, psi, theta,"                                                //  7  8  9
            " rot_x/deg/sec, rot_y/deg/sec, rot_z/deg/sec,"                    // 10 11 12
            " p_stat/Pa, alt/m, isBeeping,"                                    // 13 14 15
            " GPS_FIX, numSat, GPS_altitude, GPS_speed, GPS_course, latitude, longitude"   // 16 17 18 19 20 21
            "\n");

        LogTime = LogInterval+1;  // force immediate write of first data set
        LogActive = true;
      }
    }
  }

  /*****************************************************************************
   * log data to file if logging active
   *****************************************************************************/
  if (LogActive)
  {
    if (LogTime >= LogInterval)
    {
      float _nx, _ny, _nz;
      imu_qmi8658.getAccelerometer(_nx, _ny, _nz);
      float _rx, _ry, _rz;
      imu_qmi8658.getGyroscope(_rx, _ry, _rz);


      char loggerline[300];
      // write dataset to logfile

      snprintf(loggerline, sizeof(loggerline),
          "%02d.%02d.%04d,%02d:%02d:%02d,  %8ld,"                        // 1..3 time, date, millis
          "%5d,%5d,%5d,"                                                 // 4..6 n/10
          "%4d,%4d,%6d,"                                                 // 7..9 euler
          "%14d,%14d,%14d,"                                              // 10..12 rotation
          "%10d,%6d,%10d,"                                               // 13 14 15  pressure, altitude, beeping
          "%8d,%7d,%13d,%10d,%11d,%9d,%10d\n",                           // 16 17 18 19 20 21  GPS information
          day(), month(), year(), hour(), minute(), second(), millis(),  //  1  2  3
          lround(10*_nx), lround(10*_ny), lround(10*_nz),                //  4  5  6
          0, 0, 0,                                                       //  7  8  9
          lround(_rx), lround(_ry), lround(_rz),                         // 10 11 12
          lround(Baro_pressure()), lround(ThisAircraft.altitude), CIVA_Alarm,    // 13 14 15
          isValidGNSSFix(), gnss.satellites.value(), lround(gnss.altitude.meters()), lround(gnss.speed.kmph()), lround(gnss.course.deg()), // 16 17 18 19 20
          lround(10000000*gnss.location.lat()), lround(10000000*gnss.location.lng())  // 21 22
          );


      int result = LogFile.write(loggerline);
      if (result == 0)
      {
        Serial.printf("WriteError sectorspercluster %d bytespercluster %d bytespersector %d\n", uSD.sectorsPerCluster(), uSD.bytesPerCluster(), uSD.bytesPerSector());
        LogFile.close();
        TimeMoving = 0;
        LogActive = false;
      }
#if false
      static elapsedMillis FlushTime;
      if (FlushTime > 5000)
      {
        Serial.printf("Flush at line %d\n", lineswritten);
        #error Flushing does not work on all SD Cards!
        LogFile.flush();
        FlushTime = 0;
      }
#endif
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

#endif
