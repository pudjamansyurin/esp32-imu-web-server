#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include "Logger/SensorLogger.h"

typedef struct 
{
    sensors_vec_t magn;
    sensors_vec_t gyro;
    sensors_vec_t accl;
} sMARG_t;

typedef struct 
{
    float w;
    float x;
    float y;
    float z;
} sQuaternion_t;


class SensorBase {
public:
    SensorBase(SensorLogger& logger) : mLogger{logger} {};
    ~SensorBase() {};

    virtual void init(uint32_t count) = 0;
    virtual void wait() = 0;
    virtual void getEvent(sMARG_t* p_marg) = 0;
    virtual void update(const sMARG_t* p_marg) = 0;

    float getRoll()
    {
        return mTiltRads.roll * SENSORS_RADS_TO_DPS;
    }

    float getPitch()
    {
        return mTiltRads.pitch * SENSORS_RADS_TO_DPS;
    }

    float getYaw()
    {
        return mTiltRads.heading * SENSORS_RADS_TO_DPS;
    }

    virtual String getReport(const sMARG_t* p_marg, 
                             const sensors_vec_t* p_tilt = nullptr,
                             const sQuaternion_t* p_quat = nullptr)
    {
        JSONVar mData;

        mData["gyroX"] = String(p_marg->gyro.x);
        mData["gyroY"] = String(p_marg->gyro.y);
        mData["gyroZ"] = String(p_marg->gyro.z);
        mData["acclX"] = String(p_marg->accl.x);
        mData["acclY"] = String(p_marg->accl.y);
        mData["acclZ"] = String(p_marg->accl.z);
        mData["magnX"] = String(p_marg->magn.x);
        mData["magnY"] = String(p_marg->magn.y);
        mData["magnZ"] = String(p_marg->magn.z);

        if (nullptr != p_tilt)
        {
            mData["tiltY"] = String(p_tilt->heading);
            mData["tiltR"] = String(p_tilt->roll);
            mData["tiltP"] = String(p_tilt->pitch);
        }

        if (nullptr != p_quat)
        {
            mData["quatX"] = String(p_quat->x);
            mData["quatY"] = String(p_quat->y);
            mData["quatZ"] = String(p_quat->z);
        }

        return JSON.stringify(mData);
    }

protected:
    SensorLogger& mLogger;

    sensors_vec_t mTiltRads;

    virtual void calibrate(uint32_t count) = 0;
    
};

#endif /* SENSOR_BASE_H_ */