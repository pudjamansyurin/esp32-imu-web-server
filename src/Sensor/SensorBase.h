#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include "Logger/SensorLogger.h"

class SensorBase {
public:
    SensorBase(SensorLogger& logger) : mLogger{logger} {};
    ~SensorBase() {};

    virtual void init(uint32_t count) = 0;
    virtual void getTilt(sensors_vec_t* p_tilt) = 0;
    virtual void getEvent(sensors_vec_t* p_gyro, sensors_vec_t* p_accl) = 0;
    virtual void addReport(JSONVar& data) {};

    virtual String getReport(const sensors_vec_t* p_gyro, 
                             const sensors_vec_t* p_accl, 
                             const sensors_vec_t* p_tilt)
    {
        JSONVar mData;

        mData["gyroX"] = String(p_gyro->x);
        mData["gyroY"] = String(p_gyro->y);
        mData["gyroZ"] = String(p_gyro->z);
        mData["acclX"] = String(p_accl->x);
        mData["acclY"] = String(p_accl->y);
        mData["acclZ"] = String(p_accl->z);
        mData["tiltY"] = String(p_tilt->heading);
        mData["tiltR"] = String(p_tilt->roll);
        mData["tiltP"] = String(p_tilt->pitch);

        addReport(mData);
        return JSON.stringify(mData);
    }

protected:
    SensorLogger& mLogger;

    virtual void calibrate(uint32_t count) = 0;
    
};

#endif /* SENSOR_BASE_H_ */