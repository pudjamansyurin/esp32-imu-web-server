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
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_JSON.h>
#include "SPIFFS.h"

#ifndef USE_DMP
#include "Sensor/SensorFusion.h"
#else
#include "Sensor/SensorDMP.h"
#endif

/* private macros ------------------------------------------------------------*/
#define LED_PIN         2U
#define CALIB_CNT       10U
#define REPORT_MS       250U

/* private variables ---------------------------------------------------------*/
#ifndef USE_DMP
SensorFusion sensor(1, 0.03, 0.98);
#else
SensorDMP sensor(23);
#endif

Adafruit_SSD1306 oled(128, 32, &Wire);
AsyncWebServer server(SVR_PORT);
AsyncEventSource event(SVR_EVT);
sensors_vec_t gyro;
sensors_vec_t accl;
sensors_vec_t tilt;
uint32_t lastReport;

/* private functions delcarations --------------------------------------------*/
void log(const char *msg);
void err(const char* err);
void initOLED();
void initSPIFFS();
void initWiFi();
void report(const sensors_vec_t* p_gyro, 
            const sensors_vec_t* p_accl, 
            const sensors_vec_t* p_tilt);

/* public functions ----------------------------------------------------------*/
void setup() 
{
    pinMode(LED_PIN, OUTPUT);
    
    Wire.begin(); 
    Wire.setClock(400000);
    Serial.begin(115200);

    initOLED();
    sensor.init(CALIB_CNT);
    initSPIFFS();
    initWiFi();

    // Handle Web Server Events
    event.onConnect([](AsyncEventSourceClient *client) {
        if (client->lastId())
        {
            Serial.printf("Client %u reconnected!\n", client->lastId());
        }
        client->send("hello!", NULL, millis(), 10000);
    });

    // Handle Web Server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        memset(&tilt, 0x0, sizeof(sensors_vec_t));
        request->send(200, "text/plain", "OK");
    });
    server.on("/reset-y", HTTP_GET, [](AsyncWebServerRequest *request) {
        tilt.heading = 0;
        request->send(200, "text/plain", "OK");
    });
    server.on("/reset-r", HTTP_GET, [](AsyncWebServerRequest *request) {
        tilt.roll = 0;
        request->send(200, "text/plain", "OK");
    });
    server.on("/reset-p", HTTP_GET, [](AsyncWebServerRequest *request) {
        tilt.pitch = 0;
        request->send(200, "text/plain", "OK");
    });

    server.serveStatic("/", SPIFFS, "/");
    server.addHandler(&event);
    server.begin();
}

void loop() 
{
    sensor.getEvent(&gyro, &accl);
    sensor.getTilt(&tilt);

    // report to client
    if (REPORT_MS < (millis() - lastReport))
    {
        report(&gyro, &accl, &tilt);
        lastReport = millis();
    }
}

/* private functions definitions ---------------------------------------------*/
void log(const char *msg)
{
    Serial.println(msg);
    oled.println(msg);
    oled.display();
}

void err(const char* err) 
{
    log(err);

    while(1) 
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    };
}

void initOLED() 
{
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
    { 
        err("OLED (SSD1306) not found!");
    }

    // configure oled
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setRotation(90);
    oled.clearDisplay();
    // welcome screen
    log("Viewtrix Technology");
}

void initSPIFFS() 
{
    if (!SPIFFS.begin()) 
    {
        err("Web storage (SPIFFS) err!");
    }
}

void initWiFi() 
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID_NAME, SSID_PASS);

    log("Connecting...");
    while (WL_CONNECTED != WiFi.status()) 
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("");
    log("Connected");
}

void report(const sensors_vec_t* p_gyro, 
            const sensors_vec_t* p_accl, 
            const sensors_vec_t* p_tilt)
{
    JSONVar data;
    String json;

    // convert to json
    data["gyroX"] = String(p_gyro->x);
    data["gyroY"] = String(p_gyro->y);
    data["gyroZ"] = String(p_gyro->z);
    data["acclX"] = String(p_accl->x);
    data["acclY"] = String(p_accl->y);
    data["acclZ"] = String(p_accl->z);
    data["tiltY"] = String(p_tilt->heading);
    data["tiltR"] = String(p_tilt->roll);
    data["tiltP"] = String(p_tilt->pitch);
    json = JSON.stringify(data);

    // report to web
    event.send(json.c_str(), "readings", millis());

    // report to oled
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.printf("%s:%d\n", WiFi.localIP().toString().c_str(), SVR_PORT);
    oled.printf("Y: %3.2f deg\n", p_tilt->heading * SENSORS_RADS_TO_DPS);
    oled.printf("R: %3.2f deg\n", p_tilt->roll    * SENSORS_RADS_TO_DPS);
    oled.printf("P: %3.2f deg\n", p_tilt->pitch   * SENSORS_RADS_TO_DPS);
    oled.display();
}