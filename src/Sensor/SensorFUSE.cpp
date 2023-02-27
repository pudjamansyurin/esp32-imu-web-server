#include "SensorFUSE.h"

SensorFUSE::SensorFUSE(uint32_t freq, float fltrTau, SensorLogger& logger)
    : SensorBase{logger}
    , mFreq{freq}
    , mFltrTau{fltrTau}
{
    memset(&mTiltRads, 0x0, sizeof(sensors_vec_t));
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

    mLastTime_ms = millis();
}

void SensorFUSE::wait()
{
    uint32_t ms;

    ms = 1000.0 / (float)mFreq;
    while(ms >= (millis()-mLastTime_ms)) 
    {
        // wait blocking
    } ;
    mLastTime_ms = millis();
}

void SensorFUSE::calibrate(uint32_t count)
{
    sensors_event_t accl;
    sensors_event_t gyro;
    sensors_event_t temp;
    double sumAccl[3];
    double sumGyro[3];

    // reset to zero
    memset(sumAccl, 0x0, sizeof(sumAccl));
    memset(sumGyro, 0x0, sizeof(sumGyro));

    // get a lot of sample
    for(uint32_t u32_i = 0; u32_i < count; u32_i++) 
    {
        mpu.getEvent(&accl, &gyro, &temp);

        sumGyro[0] += gyro.gyro.x;
        sumGyro[1] += gyro.gyro.y;
        sumGyro[2] += gyro.gyro.z;
        sumAccl[0] += accl.acceleration.x;
        sumAccl[1] += accl.acceleration.y;
        sumAccl[2] += accl.acceleration.z;

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

void SensorFUSE::update(const sMARG_t* p_marg) 
{
    const sensors_vec_t* p_accl;
    const sensors_vec_t* p_gyro;
    sensors_vec_t tiltAccl;
    sensors_vec_t tiltGyro;
    float dt;

    p_accl = &(p_marg->accl);
    p_gyro = &(p_marg->gyro);

    // get tilt from gyroscope
    dt = 1.0 / (float)mFreq;
    tiltGyro.roll    = mTiltRads.roll    + p_gyro->x * dt;
    tiltGyro.pitch   = mTiltRads.pitch   + p_gyro->y * dt;

    // get tilt from accelerometer
    tiltAccl.roll    =  atan2(p_accl->y, 
                              sqrt(p_accl->x*p_accl->x + p_accl->z*p_accl->z));
    tiltAccl.pitch   = -atan2(p_accl->x, 
                              sqrt(p_accl->y*p_accl->y + p_accl->z*p_accl->z));

    // sensor fusion using complementary filter
    mTiltRads.roll   = (mFltrTau)   * (tiltGyro.roll)  + 
                       (1-mFltrTau) * (tiltAccl.roll);
    mTiltRads.pitch  = (mFltrTau)   * (tiltGyro.pitch) + 
                       (1-mFltrTau) * (tiltAccl.pitch);
    
    // undefined for yaw
    mTiltRads.heading = 0;
}

void SensorFUSE::getEvent(sMARG_t* p_marg)
{
    sensors_event_t accl;
    sensors_event_t gyro;
    sensors_event_t temp;

    // read sensor
    mpu.getEvent(&accl, &gyro, &temp);

    // get heatmap
    gyro.gyro.x         -= mBiasGyro.x;
    gyro.gyro.y         -= mBiasGyro.y;
    gyro.gyro.z         -= mBiasGyro.z;
    accl.acceleration.x -= mBiasAccl.x;
    accl.acceleration.y -= mBiasAccl.y;
    accl.acceleration.z -= mBiasAccl.z;

    // copy data
    memcpy(&(p_marg->gyro), &(gyro.gyro), sizeof(sensors_vec_t));
    memcpy(&(p_marg->accl), &(accl.acceleration), sizeof(sensors_vec_t));
}