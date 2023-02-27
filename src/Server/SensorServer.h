#ifndef SENSOR_SERVER_H_
#define SENSOR_SERVER_H_

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_Sensor.h>
#include "Logger/SensorLogger.h"
#include "SPIFFS.h"

class SensorServer {
public:
    SensorServer(uint16_t port, String event, SensorLogger& logger);
    ~SensorServer();

    void init(const char* ssid, const char *pass);
    void start();
    void report(String&& json);

private:
    AsyncWebServer mServer;
    AsyncEventSource mEvent;

    SensorLogger& mLogger;
};

#endif /* SENSOR_SERVER_H_ */