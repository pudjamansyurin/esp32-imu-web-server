#include "SensorDMP.h"

extern void log(const char *msg);
extern void err(const char* err);

SensorDMP::SensorDMP(uint8_t pin)
{
    mPin = pin;
}

SensorDMP::~SensorDMP()
{
}

void SensorDMP::init(uint32_t count)
{
    if (!mpu.testConnection()) 
    {
        err("IMU (MPU6050) not found!");
    }

    mpu.initialize();
    if (0 !=  mpu.dmpInitialize())
    {
        err("DMP initialization fail!");
    }

    log("Calibrating...");
    calibrate(count);

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // pinMode(mPin, INPUT);
    // attachInterrupt(mPin, NULL, RISING);
}

void SensorDMP::calibrate(uint32_t count)
{
    mpu.CalibrateAccel(count);
    mpu.CalibrateGyro(count);
}

void SensorDMP::getTilt(sensors_vec_t* p_tilt) 
{
    VectorFloat gravity;
    float ypr[3];

    mpu.dmpGetGravity(&gravity, &mQuat);
    mpu.dmpGetYawPitchRoll(ypr, &mQuat, &gravity);

    p_tilt->heading = ypr[0];
    p_tilt->pitch   = -ypr[1];
    p_tilt->roll    = ypr[2];
}

void SensorDMP::getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl)
{
    VectorInt16 v;

    while(!mpu.dmpGetCurrentFIFOPacket(mFifoBuf))
    {
        // wait blocking
    }

    mpu.dmpGetQuaternion(&mQuat, mFifoBuf);

    // copy data
    mpu.dmpGetAccel(&v, mFifoBuf);    // MPU6050_RANGE_2_G
    p_accl->x = ((float) v.x / 16384.0) * SENSORS_GRAVITY_STANDARD;
    p_accl->y = ((float) v.y / 16384.0) * SENSORS_GRAVITY_STANDARD;
    p_accl->z = ((float) v.z / 16384.0) * SENSORS_GRAVITY_STANDARD;
    mpu.dmpGetGyro(&v, mFifoBuf);     // MPU6050_RANGE_2000_DEG
    p_gyro->x = ((float) v.x / 16.4) * SENSORS_DPS_TO_RADS;
    p_gyro->y = ((float) v.y / 16.4) * SENSORS_DPS_TO_RADS;
    p_gyro->z = ((float) v.z / 16.4) * SENSORS_DPS_TO_RADS;

}

void SensorDMP::addReport(JSONVar& data)
{
#if 0 // quaternion data is invalid without magnetometer
    data["quatX"] = String(mQuat.x);
    data["quatY"] = String(mQuat.y);
    data["quatZ"] = String(mQuat.z);
#endif
}