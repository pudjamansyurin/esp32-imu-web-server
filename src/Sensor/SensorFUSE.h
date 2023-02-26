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
    void getTilt(sensors_vec_t* p_tilt) override;
    void getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl) override;

private:
    Adafruit_MPU6050 mpu;

    sensors_event_t mAccl;
    sensors_event_t mGyro;
    sensors_event_t mTemp;

    sensors_vec_t mBiasAccl;
    sensors_vec_t mBiasGyro;

    uint32_t mFreq;
    uint64_t mLastTime_ms;

    float mFltrTau;

    void calibrate(uint32_t count) override;
    float duration();
};

#endif /* SENSOR_FUSE_H_ */