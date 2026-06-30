#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include "stdlib.h"

/* Clamp a value between min and max */
#define pid_Data_Limit(Data, min, max)      (((Data) < (min)) ? (min) : (((Data) > (max) ? (max) : (Data))))


typedef enum
{
    PID_MODE_POSITION   = 0,            // Positional PID
    PID_MODE_ADD,                       // Incremental PID
    PID_MODE_POSITION_D_ON_MEASUREMENT, // Positional PID with derivative on measurement
}PID_MODE_TypeDef;


typedef struct
{
    PID_MODE_TypeDef    mode;           // PID mode
    float               kp;             // Proportional gain
    float               ki;             // Integral gain
    float               kd;             // Derivative gain
    float               iOutMax;        // Integral output limit
    float               outMax;         // Total output limit
}PID_InitTypeDef;


typedef struct
{
    PID_MODE_TypeDef    mode;

    float               set;            // Setpoint
    float               err3[3];        // Error history (current, prev, prev-prev)
    float               real3[3];       // Real measurement history

    float               kp;
    float               ki;
    float               kd;

    float               pOut;           // Proportional output
    float               iOut;           // Integral output
    float               dOut;           // Derivative output

    float               out;            // Total PID output

    float               iOutMax;
    float               outMax;

}PID_HandleTypeDef;

float PID_calc(PID_HandleTypeDef *pid, float exp_data, float real_data);   // Calculate PID output
void  PID_init(PID_HandleTypeDef *pid, PID_InitTypeDef *PID);              // Initialize PID with config
void  PID_cleardata(PID_HandleTypeDef *pid);                               // Reset PID internal state
void  PID_loadparam(PID_HandleTypeDef *pid, float kp, float ki, float kd); // (Not implemented here)
void  PID_loadtarget(PID_HandleTypeDef *pid, float target);                // (Not implemented here)

#endif /* __PID_H */
