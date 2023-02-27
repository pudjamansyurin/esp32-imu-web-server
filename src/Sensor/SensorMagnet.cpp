#include "SensorMagnet.h"

SensorMagnet::SensorMagnet(float declDeg, float declMin, SensorLogger& logger)
    : SensorBase{logger}
{
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    mDeclAngle = (declDeg + (declMin / 60.0)) * SENSORS_DPS_TO_RADS;
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
    sensors_event_t magn;
    float rngX[2];
    float rngY[2];
    float rngZ[2];

    // reset to zero
    memset(rngX, 0x0, sizeof(rngX));
    memset(rngY, 0x0, sizeof(rngY));
    memset(rngZ, 0x0, sizeof(rngZ));

    // get a lot of sample
    for(uint32_t u32_i = 0; u32_i < count; u32_i++) 
    {
        hmc.getEvent(&magn);

        // Determine Min / Max values
        getRange(magn.magnetic.x, rngX);
        getRange(magn.magnetic.y, rngY);
        getRange(magn.magnetic.z, rngZ);

        delay(1);
    }

    // calculcate offset
    mBias.x = (rngX[0] + rngX[1]) / 2.0;
    mBias.y = (rngY[0] + rngY[1]) / 2.0;
    mBias.z = (rngZ[0] + rngZ[1]) / 2.0;
}

void SensorMagnet::update(const sMARG_t* p_marg) 
{
    const sensors_vec_t* p_mag;
    float yaw;

    p_mag = &(p_marg->magn);

    yaw = atan2(p_mag->x, sqrt(p_mag->y*p_mag->y + p_mag->z*p_mag->z));
    
    // fix heading in current location
    yaw += mDeclAngle;
    
    // correct angle
    yaw = fixAngle(yaw);
    
    // Assing only yaw
    mTiltRads.heading = yaw;
}

void SensorMagnet::getEvent(sMARG_t* p_marg)
{
    sensors_event_t magn;

    // read sensor
    hmc.getEvent(&magn);

    // get heatmap
    magn.magnetic.x -= mBias.x;
    magn.magnetic.y -= mBias.y;
    magn.magnetic.z -= mBias.z;

    // copy data
    memcpy(&(p_marg->magn), &(magn.magnetic), sizeof(sensors_vec_t));
}

void SensorMagnet::getRange(float val, float rng[2])
{
    // get min value
    if (val < rng[0])
    {
        rng[0] = val;
    } 

    // get max value
    if (val > rng[1]) 
    {
        rng[1] = val;
    }
}

float SensorMagnet::fixAngle(float heading)
{
    // Correct for when signs are reversed.
    if (0 > heading)
    {
        heading += 2.0 * PI;
    }
        
    // Check for wrap due to addition of declination.
    else if ((2*PI) < heading)
    {
        heading -= 2.0 * PI;
    }

    return (heading);
}