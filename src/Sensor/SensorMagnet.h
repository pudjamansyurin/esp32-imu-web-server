#ifndef SENSOR_MAGNET_H_
#define SENSOR_MAGNET_H_

#include "SensorBase.h"
#include <Adafruit_HMC5883_U.h>

typedef struct
{
    float min;
    float max;
} VectorRange_t;

class SensorMagnet : public SensorBase {
public:
    SensorMagnet(float declDeg, float declMin, SensorLogger& logger);
    ~SensorMagnet();

    void init(uint32_t count) override;
    void wait() override {};
    void getEvent(sMARG_t* p_marg) override;
    void update(const sMARG_t* p_marg) override;

private:
    Adafruit_HMC5883_Unified hmc;

    sensors_vec_t mBias;
    
    float mDeclAngle;

    void calibrate(uint32_t count);

    void getRange(float val, float rng[2]);
    float fixAngle(float heading);

};

#endif /* SENSOR_MAGNET_H_ */