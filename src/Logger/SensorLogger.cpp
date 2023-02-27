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
    mSerial.print(msg);
    mOled.print(msg);
    mOled.display();
}

void SensorLogger::report(String ip, uint16_t port, sensors_vec_t* p_tilt)
{
    mOled.clearDisplay();
    mOled.setCursor(0, 0);
    mOled.printf("%s:%d\n", ip.c_str(), port);
    mOled.printf("R: %.2f deg\n", p_tilt->roll    );
    mOled.printf("P: %.2f deg\n", p_tilt->pitch   );
    mOled.printf("Y: %.2f deg\n", p_tilt->heading );
    mOled.display();

    mSerial.printf("Orientation: ");
    mSerial.printf("%f %f %f\n",    2 * p_tilt->roll    , 
                                    2 * p_tilt->pitch   , 
                                    2 * p_tilt->heading );
}