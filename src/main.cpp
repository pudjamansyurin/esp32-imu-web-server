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
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_JSON.h>
#include "SPIFFS.h"

/* private macros ------------------------------------------------------------*/
#define RESULTANT(X,Y)  (sqrt((X)*(X) + (Y)*(Y)))

#define LED_PIN         2U
#define CALIB_CNT       500U
#define MEASURE_MS      1U
#define REPORT_MS       250U
#define TAU             0.98f
#define YAW_THRES       0.03f

/* private variables ---------------------------------------------------------*/
Adafruit_SSD1306 oled(128, 32, &Wire);
AsyncWebServer server(SVR_PORT);
AsyncEventSource event(SVR_EVT);
Adafruit_MPU6050 mpu;

sensors_vec_t tilt;
sImu_t imu;

uint64_t u64_lastMeasure;
uint64_t u64_lastReport;

/* private functions delcarations --------------------------------------------*/
void errorHandler(const char* err);
void log(const char *msg);
void initOLED();
void initMPU();
void initSPIFFS();
void initWiFi();

void measure(sImu_t* p_imu);
void calibrate(sImu_t* p_imu, uint32_t u32_sample);
void calcTilt(sensors_vec_t* p_tilt, const sImu_t* p_imu, float f_dt);
String convJSON(const sensors_vec_t* p_tilt, const sImu_t* p_imu);
void report(const sensors_vec_t* p_tilt, const sImu_t* p_imu);

/* public functions ----------------------------------------------------------*/
void setup() 
{
    pinMode(LED_PIN, OUTPUT);
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
    server.on("/reset-yaw", HTTP_GET, [](AsyncWebServerRequest *request) {
        tilt.heading = 0;
        request->send(200, "text/plain", "OK");
    });
    server.on("/reset-roll", HTTP_GET, [](AsyncWebServerRequest *request) {
        tilt.roll = 0;
        request->send(200, "text/plain", "OK");
    });
    server.on("/reset-pitch", HTTP_GET, [](AsyncWebServerRequest *request) {
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

    // measure from sensor
    u32_ms = millis() - u64_lastMeasure;
    if (MEASURE_MS < u32_ms)
    {
        u64_lastMeasure = millis();
        measure(&imu);
        calcTilt(&tilt, &imu, (u32_ms * 0.001));
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
    if (!mpu.begin()) 
    {
        errorHandler("IMU (MPU6050) not found!");
    }

    log("Calibrating...");
    calibrate(&imu, CALIB_CNT);
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

void calcTilt(sensors_vec_t* p_tilt, const sImu_t* p_imu, float f_dt)
{
    const sensors_vec_t* p_accl;
    const sensors_vec_t* p_gyro;
    sensors_vec_t tiltAccl;
    sensors_vec_t tiltGyro;

    p_accl = &(p_imu->accl.acceleration);
    p_gyro = &(p_imu->gyro.gyro);

    // get tilt from accelerometer
    tiltAccl.roll    = atan2(p_accl->y,  RESULTANT(p_accl->x, p_accl->z));
    tiltAccl.pitch   = atan2(-p_accl->x, RESULTANT(p_accl->y, p_accl->z));
    // tiltAccl.heading = atan2(p_accl->z,  RESULTANT(p_accl->y, p_accl->x));

    // get tilt from gyroscope
    tiltGyro.roll    = p_tilt->roll    + p_gyro->x * f_dt;
    tiltGyro.pitch   = p_tilt->pitch   + p_gyro->y * f_dt;
    // tiltGyro.heading = p_tilt->heading + p_gyro->z * f_dt;

    // sensor fusion using complementary filter
    p_tilt->roll     = (TAU)*(tiltGyro.roll)    + (1-TAU)*(tiltAccl.roll);
    p_tilt->pitch    = (TAU)*(tiltGyro.pitch)   + (1-TAU)*(tiltAccl.pitch);
    // p_tilt->heading  = (TAU)*(tiltGyro.heading) + (1-TAU)*(tiltAccl.heading);

    // heading (yaw) only from gyroscope
    if (YAW_THRES < abs(p_gyro->z))
    {
        p_tilt->heading += p_gyro->z * f_dt;
    }
}

String convJSON(const sensors_vec_t* p_tilt, const sImu_t* p_imu)
{
    const sensors_vec_t* p_accl;
    const sensors_vec_t* p_gyro;
    JSONVar data;

    p_accl = &(p_imu->accl.acceleration);
    p_gyro = &(p_imu->gyro.gyro);

    data["temp"]      = String(p_imu->temp.temperature);
    data["gyroX"]     = String(p_gyro->x);
    data["gyroY"]     = String(p_gyro->y);
    data["gyroZ"]     = String(p_gyro->z);
    data["acclX"]     = String(p_accl->x);
    data["acclY"]     = String(p_accl->y);
    data["acclZ"]     = String(p_accl->z);
    data["tiltYaw"]   = String(p_tilt->heading);
    data["tiltRoll"]  = String(p_tilt->roll);
    data["tiltPitch"] = String(p_tilt->pitch);

    return JSON.stringify(data);
}

void calibrate(sImu_t* p_imu, uint32_t u32_sample)
{
    sSum_t* p_biasAccl;
    sSum_t* p_biasGyro;
    sImu_t tmpImu;
    sSum_t sumAccl;
    sSum_t sumGyro;

    p_biasAccl = &(p_imu->biasAccl);
    p_biasGyro = &(p_imu->biasGyro);

    // reset to zero
    memset(&sumAccl, 0x0, sizeof(sSum_t));
    memset(&sumGyro, 0x0, sizeof(sSum_t));

    // get a lot of sample
    for(uint32_t u32_i = 0; u32_i < u32_sample; u32_i++) 
    {
        mpu.getEvent(&(tmpImu.accl), &(tmpImu.gyro), &(tmpImu.temp));

        sumAccl.d_x += tmpImu.accl.acceleration.x;
        sumAccl.d_y += tmpImu.accl.acceleration.y;
        sumAccl.d_z += tmpImu.accl.acceleration.z;
        sumGyro.d_x += tmpImu.gyro.gyro.x;
        sumGyro.d_y += tmpImu.gyro.gyro.y;
        sumGyro.d_z += tmpImu.gyro.gyro.z;

        delay(1);
    }

    // calculcate baseline
    p_biasAccl->d_x = sumAccl.d_x / u32_sample;
    p_biasAccl->d_y = sumAccl.d_y / u32_sample;
    p_biasAccl->d_z = sumAccl.d_z / u32_sample;
    p_biasGyro->d_x = sumGyro.d_x / u32_sample;
    p_biasGyro->d_y = sumGyro.d_y / u32_sample;
    p_biasGyro->d_z = sumGyro.d_z / u32_sample;

    // assume z-axis is perpendicular to earth gravity
    p_biasAccl->d_z -= SENSORS_GRAVITY_STANDARD;
}

void measure(sImu_t* p_imu)
{
    // read sensor
    mpu.getEvent(&(p_imu->accl), &(p_imu->gyro), &(p_imu->temp));

    // get heatmap
    p_imu->accl.acceleration.x -= p_imu->biasAccl.d_x;
    p_imu->accl.acceleration.y -= p_imu->biasAccl.d_y;
    p_imu->accl.acceleration.z -= p_imu->biasAccl.d_z;
    p_imu->accl.gyro.x -= p_imu->biasGyro.d_x;
    p_imu->accl.gyro.y -= p_imu->biasGyro.d_y;
    p_imu->accl.gyro.z -= p_imu->biasGyro.d_z;
}

void report(const sensors_vec_t* p_tilt, const sImu_t* p_imu)
{
    // convert to json
    const char* p_json = convJSON(p_tilt, p_imu).c_str();

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