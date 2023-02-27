#include "SensorServer.h"

SensorServer::SensorServer(uint16_t port, String event, SensorLogger& logger) 
    : mServer{port}
    , mEvent{event}
    , mLogger{logger}
{
}

SensorServer::~SensorServer()
{
}

void SensorServer::init(const char* ssid, const char *pass)
{
    // initialize server storage
    if (!SPIFFS.begin()) 
    {
        throw ("SPIFFS error\n");
    }

    // initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    // wait until connected
    mLogger.write("Connecting WiFi...");
    while (WL_CONNECTED != WiFi.status()) 
    {
        mLogger.write(".");
        delay(1000);
    }
    mLogger.write("\n");

    mLogger.write("Connected.\n");
}

void SensorServer::start()
{
    // Handle Web Server
    mServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    // Handle Web Server Events
    mEvent.onConnect([&](AsyncEventSourceClient *client) {
        if (client->lastId())
        {
            std::string str;
            str = "Recon: " + std::to_string(client->lastId()) + "\n";
            mLogger.write(str.c_str());
        }
        client->send("hello!", NULL, millis(), 10000);
    });

    mServer.serveStatic("/", SPIFFS, "/");
    mServer.addHandler(&mEvent);
    mServer.begin();
}

void SensorServer::report(String&& json)
{
    mEvent.send(json.c_str(), "readings", millis());
}