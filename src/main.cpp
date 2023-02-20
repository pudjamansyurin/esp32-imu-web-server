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

/* private macros ------------------------------------------------------------*/
#define LED_PIN         2U
#define CALIB_CNT       10U
#define REPORT_MS       250U

/* private variables ---------------------------------------------------------*/
SensorLogger logger(Serial, Wire);
SensorServer server(SVR_PORT, SVR_EVT, logger);

#ifndef USE_DMP
#include "Sensor/SensorFusion.h"
SensorBase&& sensor = SensorFusion(1, 0.01, 0.98, logger);
#else
#include "Sensor/SensorDMP.h"
SensorBase&& sensor = SensorDMP(23, logger);
#endif

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
        // initalize sensor
        sensor.init(CALIB_CNT);

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
    sensor.getEvent(&gyro, &accl);
    sensor.getTilt(&tilt);

    // report to client
    if (REPORT_MS < (millis() - lastReport))
    {
        server.report(sensor.getReport(&gyro, &accl, &tilt));
        logger.report(WiFi.localIP().toString(), SVR_PORT, &tilt);

        lastReport = millis();
    }
}