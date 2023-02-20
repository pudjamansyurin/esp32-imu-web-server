#include "SensorLogger.h"

SensorLogger::SensorLogger(HardwareSerial& serial, TwoWire& wire) 
    : mOled{128, 32, &wire}
    , mSerial{serial}
{
}

SensorLogger::~SensorLogger()
{
}

void SensorLogger::init(uint32_t baud, const char* msg)
{
    // initialize serial
    mSerial.begin(baud);

    // initialize oled
    if (!mOled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
    { 
        mSerial.println("OLED (SSD1306) not found!");
    }

    // configure
    mOled.setTextSize(1);
    mOled.setTextColor(WHITE);
    mOled.setRotation(90);
    mOled.clearDisplay();

    // welcome message
    write(msg);
}

void SensorLogger::write(const char* msg)
{
    Serial.println(msg);
    mOled.print(msg);
    mOled.display();
}

void SensorLogger::report(String ip, uint16_t port, sensors_vec_t* p_tilt)
{
    mOled.clearDisplay();
    mOled.setCursor(0, 0);
    mOled.printf("%s:%d\n", ip.c_str(), port);
    mOled.printf("Y: %3.2f deg\n", p_tilt->heading * SENSORS_RADS_TO_DPS);
    mOled.printf("R: %3.2f deg\n", p_tilt->roll    * SENSORS_RADS_TO_DPS);
    mOled.printf("P: %3.2f deg\n", p_tilt->pitch   * SENSORS_RADS_TO_DPS);
    mOled.display();
}