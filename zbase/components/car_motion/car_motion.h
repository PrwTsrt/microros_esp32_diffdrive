#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "motor.h"

// Car chassis wheel spacing, unit :m
#define ROBOT_WIDTH                  (0.135f)
#define ROBOT_LENGTH                 (0.095f)

// Half of the distance between the upper and lower wheels and the left and right wheels of the car.
#define ROBOT_APB                    (0.121f)


#define ROBOT_SPIN_SCALE             (5.0f)


typedef enum _motion_state {
    MOTION_STOP = 0,
    MOTION_RUN,
    MOTION_BACK,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_SPIN_LEFT,
    MOTION_SPIN_RIGHT,
    MOTION_BRAKE,

    MOTION_MAX_STATE
} motion_state_t;



typedef struct _car_motion
{
    float Vx;
    float Vy;
    float Wz;
} car_motion_t;



void Motion_Stop(uint8_t brake);
void Motion_Ctrl(float V_x, float V_y, float V_z);
void Motion_Ctrl_State(uint8_t state, float speed);
void Motion_Get_Speed(car_motion_t* car);


void Motion_Init(void);


#ifdef __cplusplus
}
#endif
