#include "SensorBase.h"
#include <Adafruit_MPU6050.h>

typedef struct
{
    double x;
    double y;
    double z;
} VectorDouble_t;


class SensorFusion: public SensorBase {
public:
    SensorFusion(uint32_t deltaTime, float yawThres, float fltrTau);
    ~SensorFusion();

    void init(uint32_t count) override;
    void getTilt(sensors_vec_t* p_tilt) override;
    void getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl) override;

private:
    Adafruit_MPU6050 mpu;

    sensors_event_t mAccl;
    sensors_event_t mGyro;
    sensors_event_t mTemp;

    sensors_vec_t mBiasAccl;
    sensors_vec_t mBiasGyro;

    uint32_t mDeltaTime;
    uint32_t mLastTime;

    float mFltrTau;
    float mYawThres;

    void calibrate(uint32_t count) override;
};