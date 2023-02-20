#include <Adafruit_Sensor.h>

class SensorBase {
public:
    // SensorBase() {};
    ~SensorBase() {};

    virtual void init(uint32_t count) = 0;
    virtual void getTilt(sensors_vec_t* p_tilt) = 0;
    virtual void getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl) = 0;

protected:
    virtual void calibrate(uint32_t count) = 0;
};