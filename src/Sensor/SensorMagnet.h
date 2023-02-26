#ifndef SENSOR_MAGNET_H_
#define SENSOR_MAGNET_H_

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "Logger/SensorLogger.h"

typedef struct
{
    float min;
    float max;
} VectorRange_t;

class SensorMagnet {
public:
    SensorMagnet(float declDeg, float declMin, SensorLogger& logger);
    ~SensorMagnet();

    void init(uint32_t count);
    void getTilt(sensors_vec_t* p_tilt);
    void getEvent(sensors_vec_t* p_mag);

private:
    Adafruit_HMC5883_Unified hmc;
    SensorLogger& mLogger;

    sensors_event_t mMag;
    sensors_vec_t mBias;
    
    float mDeclAngle;

    void calibrate(uint32_t count);

    void getRange(float val, float rng[2]);
    float fixAngle(float heading);

};

#endif /* SENSOR_MAGNET_H_ */