#include "SensorFusion.h"

SensorFusion::SensorFusion(uint32_t scanTime_ms, float yawThres, float fltrTau, SensorLogger& logger)
    : SensorBase{logger}
    , mScanTime_ms{scanTime_ms}
    , mYawThres{yawThres}
    , mFltrTau{fltrTau}
    , mLastTime_ms{0}
    , mElapsedTime_ms{0}
{
}

SensorFusion::~SensorFusion()
{
}

void SensorFusion::init(uint32_t count)
{
    if (!mpu.begin()) 
    {
        throw ("MPU6050 error\n");
    }

    mLogger.write("Calibrating...\n");
    calibrate(count);
}

void SensorFusion::calibrate(uint32_t count)
{
    VectorDouble_t sumAccl;
    VectorDouble_t sumGyro;

    // reset to zero
    memset(&sumAccl, 0x0, sizeof(VectorDouble_t));
    memset(&sumGyro, 0x0, sizeof(VectorDouble_t));

    // get a lot of sample
    for(uint32_t u32_i = 0; u32_i < count; u32_i++) 
    {
        mpu.getEvent(&mAccl, &mGyro, &mTemp);

        sumGyro.x += mGyro.gyro.x;
        sumGyro.y += mGyro.gyro.y;
        sumGyro.z += mGyro.gyro.z;
        sumAccl.x += mAccl.acceleration.x;
        sumAccl.y += mAccl.acceleration.y;
        sumAccl.z += mAccl.acceleration.z;

        delay(1);
    }

    // calculcate baseline
    mBiasGyro.x = sumGyro.x / count;
    mBiasGyro.y = sumGyro.y / count;
    mBiasGyro.z = sumGyro.z / count;
    mBiasAccl.x = sumAccl.x / count;
    mBiasAccl.y = sumAccl.y / count;
    mBiasAccl.z = sumAccl.z / count;

    // assume z-axis is perpendicular to earth gravity
    mBiasAccl.z -= SENSORS_GRAVITY_STANDARD;
}

void SensorFusion::getTilt(sensors_vec_t* p_tilt) 
{
    const sensors_vec_t* p_accl;
    const sensors_vec_t* p_gyro;
    sensors_vec_t tiltAccl;
    sensors_vec_t tiltGyro;
    float dt;

    p_accl = &(mAccl.acceleration);
    p_gyro = &(mGyro.gyro);

    // get tilt from accelerometer
    tiltAccl.roll    =  atan2(p_accl->y, sqrt(p_accl->x*p_accl->x + p_accl->z*p_accl->z));
    tiltAccl.pitch   = -atan2(p_accl->x, sqrt(p_accl->y*p_accl->y + p_accl->z*p_accl->z));

    // get tilt from gyroscope
    dt = (float)mScanTime_ms * 0.001;
    tiltGyro.roll    = p_tilt->roll    + p_gyro->x * dt;
    tiltGyro.pitch   = p_tilt->pitch   + p_gyro->y * dt;

    // sensor fusion using complementary filter
    p_tilt->roll     = (mFltrTau)*(tiltGyro.roll)  + (1-mFltrTau)*(tiltAccl.roll);
    p_tilt->pitch    = (mFltrTau)*(tiltGyro.pitch) + (1-mFltrTau)*(tiltAccl.pitch);

    // heading (yaw) only from gyroscope
    if (mYawThres < abs(p_gyro->z))
    {
        p_tilt->heading += p_gyro->z * dt;
    }
}

void SensorFusion::getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl)
{
    do {
        // wait blocking
        mElapsedTime_ms = millis()-mLastTime_ms;
    } while(mScanTime_ms > mElapsedTime_ms);
    mLastTime_ms = millis();

    // read sensor
    mpu.getEvent(&mAccl, &mGyro, &mTemp);

    // get heatmap
    mGyro.gyro.x         -= mBiasGyro.x;
    mGyro.gyro.y         -= mBiasGyro.y;
    mGyro.gyro.z         -= mBiasGyro.z;
    mAccl.acceleration.x -= mBiasAccl.x;
    mAccl.acceleration.y -= mBiasAccl.y;
    mAccl.acceleration.z -= mBiasAccl.z;

    // copy data
    memcpy(p_gyro, &(mGyro.gyro), sizeof(sensors_vec_t));
    memcpy(p_accl, &(mAccl.acceleration), sizeof(sensors_vec_t));
}