#ifndef SENSOR_LOGGER_H_
#define SENSOR_LOGGER_H_

#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

class SensorLogger {
public:
    SensorLogger(HardwareSerial& serial, TwoWire& wire);
    ~SensorLogger();

    void init(uint32_t baud, const char* msg);
    void write(const char *msg);
    void report(String ip, uint16_t port, sensors_vec_t* p_tilt);

private:
    Adafruit_SSD1306 mOled;
    HardwareSerial& mSerial;
};

#endif /* SENSOR_LOGGER_H_ */