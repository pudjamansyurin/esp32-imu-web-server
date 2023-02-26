#include "SensorFUSE.h"

SensorFUSE::SensorFUSE(uint32_t freq, float fltrTau, SensorLogger& logger)
    : SensorBase{logger}
    , mFreq{freq}
    , mFltrTau{fltrTau}
    , mLastTime_ms{0}
{
}

SensorFUSE::~SensorFUSE()
{
}

void SensorFUSE::init(uint32_t count)
{
    if (!mpu.begin()) 
    {
        throw ("MPU6050 error\n");
    }

    mLogger.write("Calibrating MPU...\n");
    calibrate(count);
}

void SensorFUSE::wait()
{
    uint32_t ms;

    ms = duration() * 1000;
    while(ms > (millis()-mLastTime_ms)) 
    {
        // wait blocking
    } ;
    mLastTime_ms = millis();
}

void SensorFUSE::calibrate(uint32_t count)
{
    double sumAccl[3];
    double sumGyro[3];

    // reset to zero
    memset(sumAccl, 0x0, sizeof(sumAccl));
    memset(sumGyro, 0x0, sizeof(sumGyro));

    // get a lot of sample
    for(uint32_t u32_i = 0; u32_i < count; u32_i++) 
    {
        mpu.getEvent(&mAccl, &mGyro, &mTemp);

        sumGyro[0] += mGyro.gyro.x;
        sumGyro[1] += mGyro.gyro.y;
        sumGyro[2] += mGyro.gyro.z;
        sumAccl[0] += mAccl.acceleration.x;
        sumAccl[1] += mAccl.acceleration.y;
        sumAccl[2] += mAccl.acceleration.z;

        delay(1);
    }

    // calculcate baseline
    mBiasGyro.x = sumGyro[0] / count;
    mBiasGyro.y = sumGyro[1] / count;
    mBiasGyro.z = sumGyro[2] / count;
    mBiasAccl.x = sumAccl[0] / count;
    mBiasAccl.y = sumAccl[1] / count;
    mBiasAccl.z = sumAccl[2] / count;

    // assume z-axis is perpendicular to earth gravity
    mBiasAccl.z -= SENSORS_GRAVITY_STANDARD;
}

void SensorFUSE::getTilt(sensors_vec_t* p_tilt) 
{
    const sensors_vec_t* p_accl;
    const sensors_vec_t* p_gyro;
    sensors_vec_t tiltAccl;
    sensors_vec_t tiltGyro;
    float dt;

    p_accl = &(mAccl.acceleration);
    p_gyro = &(mGyro.gyro);

    dt = duration();
    
    // get tilt from accelerometer
    tiltAccl.roll    =  atan2(p_accl->y, sqrt(p_accl->x*p_accl->x + p_accl->z*p_accl->z));
    tiltAccl.pitch   = -atan2(p_accl->x, sqrt(p_accl->y*p_accl->y + p_accl->z*p_accl->z));

    // get tilt from gyroscope
    tiltGyro.roll    = p_tilt->roll    + p_gyro->x * dt;
    tiltGyro.pitch   = p_tilt->pitch   + p_gyro->y * dt;

    // sensor fusion using complementary filter
    p_tilt->roll     = (mFltrTau)*(tiltGyro.roll)  + (1-mFltrTau)*(tiltAccl.roll);
    p_tilt->pitch    = (mFltrTau)*(tiltGyro.pitch) + (1-mFltrTau)*(tiltAccl.pitch);
}

void SensorFUSE::getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl)
{
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

float SensorFUSE::duration()
{
    return 1.0 / (float)mFreq;
}