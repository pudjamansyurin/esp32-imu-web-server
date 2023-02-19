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
#include <Adafruit_SSD1306.h>
#include <Arduino_JSON.h>
#include "SPIFFS.h"

#include <MPU6050_6Axis_MotionApps612.h>

/* private macros ------------------------------------------------------------*/
#define LED_PIN         2U
#define INT_PIN         23U
#define CALIB_CNT       6U
#define REPORT_MS       250U

/* private variables ---------------------------------------------------------*/
Adafruit_SSD1306 oled(128, 32, &Wire);
AsyncWebServer server(SVR_PORT);
AsyncEventSource event(SVR_EVT);

MPU6050 mpu;
sensors_vec_t tilt;
sImu_t imu;

// MPU control/status vars
uint8_t fifoBuffer[64]; // FIFO storage buffer

uint64_t u64_lastReport;

/* private functions delcarations --------------------------------------------*/
void errorHandler(const char* err);
void log(const char *msg);
void initOLED();
void initMPU();
void initSPIFFS();
void initWiFi();
String json(const sensors_vec_t* p_tilt, const sImu_t* p_imu);
void report(const sensors_vec_t* p_tilt, const sImu_t* p_imu);

/* ISR functions -------------------------------------------------------------*/
void dmpDataReady() 
{
    // not used, already polling
}

/* public functions ----------------------------------------------------------*/
void setup() 
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(INT_PIN, INPUT);

    Wire.begin(); 
    Wire.setClock(400000);
    Serial.begin(115200);

    initSPIFFS();
    initOLED();
    initMPU();
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
    uint32_t u32_ms;

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
    { 
        // orientation/motion vars
        Quaternion q;           // quaternion container
        VectorFloat gravity;    // gravity vector
        float ypr[3];           // yaw/pitch/roll container and gravity vector

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        VectorInt16 aa;         // accel sensor measurements
        VectorInt16 aaReal;     // gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // world-frame accel sensor measurements
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        memset(&imu, 0x0, sizeof(sImu_t));
        tilt.heading = ypr[0];
        tilt.pitch   = -ypr[1];
        tilt.roll    = ypr[2];
    }

    // report to client
    u32_ms = millis() - u64_lastReport;
    if (REPORT_MS < u32_ms)
    {
        u64_lastReport = millis();
        report(&tilt, &imu);
    }
}

/* private functions definitions ---------------------------------------------*/
void errorHandler(const char* err) 
{
    log(err);

    while(1) 
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    };
}

void log(const char *msg)
{
    Serial.println(msg);
    oled.println(msg);
    oled.display();
}

void initOLED() 
{
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
    { 
        errorHandler("OLED (SSD1306) not found!");
    }

    // configure oled
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setRotation(90);
    oled.clearDisplay();
    // welcome screen
    log("Viewtrix Technology");
}

void initMPU()
{
    if (!mpu.testConnection()) 
    {
        errorHandler("IMU (MPU6050) not found!");
    }

    mpu.initialize();
    if (0 !=  mpu.dmpInitialize())
    {
        errorHandler("DMP initialization fail!");
    }

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(INT_PIN, dmpDataReady, RISING);
}

void initSPIFFS() 
{
    if (!SPIFFS.begin()) 
    {
        errorHandler("Web storage (SPIFFS) error!");
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

String json(const sensors_vec_t* p_tilt, const sImu_t* p_imu)
{
    const sensors_vec_t* p_accl;
    const sensors_vec_t* p_gyro;
    JSONVar data;

    p_accl = &(p_imu->accl.acceleration);
    p_gyro = &(p_imu->gyro.gyro);

    data["temp"]  = String(p_imu->temp.temperature);
    data["gyroX"] = String(p_gyro->x);
    data["gyroY"] = String(p_gyro->y);
    data["gyroZ"] = String(p_gyro->z);
    data["acclX"] = String(p_accl->x);
    data["acclY"] = String(p_accl->y);
    data["acclZ"] = String(p_accl->z);
    data["tiltY"] = String(p_tilt->heading);
    data["tiltR"] = String(p_tilt->roll);
    data["tiltP"] = String(p_tilt->pitch);
#if 0
    data["quatW"] = String(q.w);
    data["quatX"] = String(q.x);
    data["quatY"] = String(q.y);
    data["quatZ"] = String(q.z);
#endif

    return JSON.stringify(data);
}

void report(const sensors_vec_t* p_tilt, const sImu_t* p_imu)
{
    // convert to json
    const char* p_json = json(p_tilt, p_imu).c_str();

    // report to web
    event.send(p_json, "readings", millis());

    // report to oled
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.printf("%s:%d\n", WiFi.localIP().toString().c_str(), SVR_PORT);
    oled.printf("Y: %3.2f deg\n", p_tilt->heading * SENSORS_RADS_TO_DPS);
    oled.printf("R: %3.2f deg\n", p_tilt->roll    * SENSORS_RADS_TO_DPS);
    oled.printf("P: %3.2f deg\n", p_tilt->pitch   * SENSORS_RADS_TO_DPS);
    oled.display();
}