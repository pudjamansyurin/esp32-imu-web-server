#include "SensorDMP.h"

SensorDMP::SensorDMP(uint8_t pin, SensorLogger& logger)
    : SensorBase{logger}
    , mPin{pin}
{
}

SensorDMP::~SensorDMP()
{
}

void SensorDMP::init(uint32_t count)
{
    if (!mpu.testConnection()) 
    {
        throw ("MPU6050 error\n");
    }

    mpu.initialize();
    if (0 !=  mpu.dmpInitialize())
    {
        throw ("DMP error\n");
    }

    mLogger.write("Calibrating MPU...\n");
    calibrate(count);

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // pinMode(mPin, INPUT);
    // attachInterrupt(mPin, NULL, RISING);
}

void SensorDMP::wait()
{
    while(!mpu.dmpGetCurrentFIFOPacket(mFifoBuf))
    {
        // wait blocking
    }
}

void SensorDMP::calibrate(uint32_t count)
{
    count /= 100;
    mpu.CalibrateAccel(count);
    mpu.CalibrateGyro(count);
}

void SensorDMP::update(const sMARG_t* p_marg) 
{
    VectorFloat gravity;
    float ypr[3];

    mpu.dmpGetGravity(&gravity, &mQuat);
    mpu.dmpGetYawPitchRoll(ypr, &mQuat, &gravity);

    mTiltRads.heading =  ypr[0];
    mTiltRads.pitch   = -ypr[1];
    mTiltRads.roll    =  ypr[2];
}

void SensorDMP::getEvent(sMARG_t* p_marg)
{
    VectorInt16 v;

    mpu.dmpGetQuaternion(&mQuat, mFifoBuf);

    // copy data
    mpu.dmpGetAccel(&v, mFifoBuf);    // MPU6050_RANGE_2_G
    p_marg->accl.x = ((float) v.x / 16384.0) * SENSORS_GRAVITY_STANDARD;
    p_marg->accl.y = ((float) v.y / 16384.0) * SENSORS_GRAVITY_STANDARD;
    p_marg->accl.z = ((float) v.z / 16384.0) * SENSORS_GRAVITY_STANDARD;
    mpu.dmpGetGyro(&v, mFifoBuf);     // MPU6050_RANGE_2000_DEG
    p_marg->gyro.x = ((float) v.x / 16.4) * SENSORS_DPS_TO_RADS;
    p_marg->gyro.y = ((float) v.y / 16.4) * SENSORS_DPS_TO_RADS;
    p_marg->gyro.z = ((float) v.z / 16.4) * SENSORS_DPS_TO_RADS;

}