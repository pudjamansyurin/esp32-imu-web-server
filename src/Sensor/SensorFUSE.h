#ifndef SENSOR_FUSE_H_
#define SENSOR_FUSE_H_

#include "SensorBase.h"
#include <Adafruit_MPU6050.h>

class SensorFUSE: public SensorBase {
public:
    SensorFUSE(uint32_t freq, float fltrTau, SensorLogger& logger);
    ~SensorFUSE();

    void init(uint32_t count) override;
    void wait() override;
    void getEvent(sMARG_t* p_marg) override;
    void update(const sMARG_t* p_marg) override;

private:
    Adafruit_MPU6050 mpu;

    sensors_vec_t mBiasAccl;
    sensors_vec_t mBiasGyro;

    uint32_t mFreq;
    uint32_t mLastTime_ms;

    float mFltrTau;

    void calibrate(uint32_t count) override;
};

#endif /* SENSOR_FUSE_H_ */