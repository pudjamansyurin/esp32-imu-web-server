/*********
  Rui Santos
  Complete project details at 
  https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include "main.h"
#include <Wire.h>
#include "Logger/SensorLogger.h"
#include "Server/SensorServer.h"
#include "Sensor/SensorMagnet.h"

/* private macros ------------------------------------------------------------*/
#define LED_PIN         2
#define MPU_PIN         23

#define CALIB_CNT       200
#define REPORT_MS       250
#define SAMPLE_HZ       100

/* private variables ---------------------------------------------------------*/
SensorLogger logger(Serial, Wire);
SensorServer server(SVR_PORT, SVR_EVT, logger);

#ifndef USE_DMP
#include "Sensor/SensorFUSE.h"
SensorBase&& mpu = SensorFUSE(SAMPLE_HZ, 0.98, logger);
#else
#include "Sensor/SensorDMP.h"
SensorBase&& mpu = SensorDMP(MPU_PIN, logger);
#endif

// For Sukun Malang declination angle is +0'46E 
SensorMagnet hmc(0.0, 46.0, logger);

#ifdef USE_AHRS
#include <Adafruit_AHRS.h>
// pick your filter! slower == better quality output
// Adafruit_NXPSensorFusion filter; // slowest
// Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter;  // fastest/smallest
#endif

sMARG_t marg;
sensors_vec_t tilt;
sQuaternion_t quat;
uint32_t lastReport;

/* public functions ----------------------------------------------------------*/
void setup() 
{
    pinMode(LED_PIN, OUTPUT);
    
    // i2c periph init
    Wire.begin(); 
    Wire.setClock(400000);

    // initalize logger
    logger.init(115200, "Viewtrix Technology\n");

    try
    {
        // initalize imu sensor
        mpu.init(CALIB_CNT);

        /* initalize magnetic sensor */
        hmc.init(CALIB_CNT);

        // initialize server
        server.init(SSID_NAME, SSID_PASS);
        server.start();
    }
    catch(char const *error)
    {
        logger.write(error);
        while(1) 
        {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        };
    }

#ifdef USE_AHRS
    filter.begin(SAMPLE_HZ);
#endif
}

void loop() 
{
    // wait until sample time
    mpu.wait();

    // get sensor events
    hmc.getEvent(&marg);
    mpu.getEvent(&marg);

    // update current position
#ifndef USE_AHRS
    hmc.update(&marg);
    mpu.update(&marg);
#else
    // filter want gyro in dps
    marg.gyro.x *= SENSORS_RADS_TO_DPS;
    marg.gyro.y *= SENSORS_RADS_TO_DPS;
    marg.gyro.z *= SENSORS_RADS_TO_DPS;

    // update filter
    filter.update(marg.gyro.x, marg.gyro.y, marg.gyro.z, 
                  marg.accl.x, marg.accl.y, marg.accl.z, 
                  marg.magn.x, marg.magn.y, marg.magn.z);
#endif

    // reporting
    if (REPORT_MS < (millis() - lastReport))
    {
        lastReport = millis();
        
        // get tilt information
#ifndef USE_AHRS
        tilt.roll    = mpu.getRoll();
        tilt.pitch   = mpu.getPitch();
        tilt.heading = hmc.getYaw();
#else
        tilt.roll    = filter.getRoll();
        tilt.pitch   = filter.getPitch();
        tilt.heading = filter.getYaw();
#endif

        // report 
        logger.report(WiFi.localIP().toString(), SVR_PORT, &tilt);
        server.report(mpu.getReport(&marg, &tilt));
    }
}