#include "SensorBase.h"
#include <MPU6050_6Axis_MotionApps612.h>

class SensorDMP: public SensorBase {
public:
    SensorDMP(uint8_t pin);
    ~SensorDMP();

    void init(uint32_t count) override;
    void getTilt(sensors_vec_t* p_tilt) override;
    void getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl) override;
    void addReport(JSONVar& data) override;

private:
    MPU6050 mpu;
    uint8_t mPin;
    uint8_t mFifoBuf[64];
    Quaternion mQuat;

    void calibrate(uint32_t count) override;

};