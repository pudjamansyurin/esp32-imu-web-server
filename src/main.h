#ifndef MAIN_H_
#define MAIN_H_

#define CONFIG_DISABLE_HAL_LOCKS 1 

#include <Adafruit_Sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* exported macros ------------------------------------------------------------*/

/* exported typedefs ---------------------------------------------------------*/
typedef struct
{
    double d_x;
    double d_y;
    double d_z;
} sSum_t;

typedef struct 
{
    sensors_event_t accl;       /* accelerometer */
    sensors_event_t gyro;       /* gyroscope */
    sensors_event_t temp;       /* temperature */
    sSum_t biasAccl;
    sSum_t biasGyro;
} sImu_t;


#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */