#include "SensorMagnet.h"

SensorMagnet::SensorMagnet(float declDeg, float declMin, SensorLogger& logger)
    : mLogger{logger}
{
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    mDeclAngle = (declDeg + (declMin / 60.0)) / (180.0 / PI);
}

SensorMagnet::~SensorMagnet()
{
}

void SensorMagnet::init(uint32_t count)
{
    if (!hmc.begin()) 
    {
        throw ("HMC5883L error\n");
    }

    mLogger.write("Calibrating HMC...\n");
    calibrate(count);
}

void SensorMagnet::calibrate(uint32_t count)
{
    float rngX[2];
    float rngY[2];

    // reset to zero
    memset(rngX, 0x0, sizeof(rngX));
    memset(rngY, 0x0, sizeof(rngY));

    // get a lot of sample
    for(uint32_t u32_i = 0; u32_i < count; u32_i++) 
    {
        hmc.getEvent(&mMag);

        // Determine Min / Max values
        getRange(mMag.magnetic.x, rngX);
        getRange(mMag.magnetic.y, rngY);

        delay(1);
    }

    // calculcate offset
    mBias.x = (rngX[0] + rngX[1]) / 2;
    mBias.y = (rngY[0] + rngY[1]) / 2;
}

void SensorMagnet::getTilt(sensors_vec_t* p_tilt) 
{
    const sensors_vec_t* p_mag;
    float yaw;

    p_mag = &(mMag.magnetic);

    // Hold the module so that Z is pointing 'up',
    // and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, 
    // then correct for signs of axis.
    yaw = atan2(p_mag->x, sqrt(p_mag->y*p_mag->y + p_mag->z*p_mag->z));
    
    // fix heading in current location
    yaw += mDeclAngle;
    
    // correct angle
    yaw = fixAngle(yaw);
    
    // Assing only yaw
    p_tilt->heading = yaw;
}

void SensorMagnet::getEvent(sensors_vec_t* p_mag)
{
    // read sensor
    hmc.getEvent(&mMag);

    // get heatmap
    mMag.magnetic.x -= mBias.x;
    mMag.magnetic.y -= mBias.y;

    // copy data
    memcpy(p_mag, &(mMag.magnetic), sizeof(sensors_vec_t));
}

void SensorMagnet::getRange(float val, float rng[2])
{
    // get min value
    if (val < rng[0])
    {
        rng[0] = val;
    } 

    // get max value
    else if (val > rng[1]) 
    {
        rng[1] = val;
    }
}

float SensorMagnet::fixAngle(float heading)
{
    // Correct for when signs are reversed.
    if (0 > heading)
    {
        heading += 2*PI;
    }
        
    // Check for wrap due to addition of declination.
    else if ((2*PI) < heading)
    {
        heading -= 2*PI;
    }

    return (heading);
}