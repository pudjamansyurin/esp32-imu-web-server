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
#define MPU_PIN         23U
#define LED_PIN         2U

#define CALIB_CNT       200U
#define REPORT_MS       250U

/* private variables ---------------------------------------------------------*/
SensorLogger logger(Serial, Wire);
SensorServer server(SVR_PORT, SVR_EVT, logger);

#ifndef USE_DMP
#include "Sensor/SensorFUSE.h"
SensorBase&& mpu = SensorFUSE(200, 0.98, logger);
#else
#include "Sensor/SensorDMP.h"
SensorBase&& mpu = SensorDMP(MPU_PIN, logger);
#endif

// For Sukun Malang declination angle is +0'46E 
SensorMagnet hmc(0.0, 46.0, logger);

sensors_vec_t mag;
sensors_vec_t gyro;
sensors_vec_t accl;
sensors_vec_t tilt;
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
        server.start(tilt);
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
}

void loop() 
{
    sensors_event_t event; 

    // wait until scan time
    mpu.wait();

    // get sensor events
    hmc.getEvent(&mag);
    mpu.getEvent(&gyro, &accl);

    // get tilt
    hmc.getTilt(&tilt);
    mpu.getTilt(&tilt);

    // report to client
    if (REPORT_MS < (millis() - lastReport))
    {
        
        server.report(mpu.getReport(&gyro, &accl, &tilt));
        logger.report(WiFi.localIP().toString(), SVR_PORT, &tilt);

        lastReport = millis();
    }
}