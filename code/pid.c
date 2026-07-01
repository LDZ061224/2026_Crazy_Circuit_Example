/*
    Use this as a library, no need to modify.
    PID controller implementation supporting three modes:
      - Positional PID
      - Positional PID with derivative on measurement
      - Incremental PID
*/

#include "pid.h"

/* Calculate PID output based on expected and actual values */
float PID_calc(PID_HandleTypeDef *pid, float exp_data, float real_data)
{
    if(pid == NULL)
    {
        return 0.0f;
    }

    if(pid->mode == PID_MODE_POSITION)
    {
        pid->set = exp_data;

        pid->err3[2] = pid->err3[1];
        pid->err3[1] = pid->err3[0];
        pid->err3[0] = exp_data - real_data;// Error calculation

        pid->pOut = pid->kp * pid->err3[0];
        pid->iOut += pid->ki * pid->err3[0];         // Accumulate integral of error
        pid->dOut = pid->kd * (pid->err3[0] - pid->err3[1]);

        if(pid->iOutMax != 0)// Apply integral limit
        {
            pid->iOut = pid_Data_Limit(pid->iOut, -(pid->iOutMax), pid->iOutMax);
        }

        pid->out = pid->pOut + pid->iOut + pid->dOut;

        if(pid->outMax != 0)// Apply output limit
        {
            pid->out = pid_Data_Limit(pid->out, -(pid->outMax), pid->outMax);
        }
    }
    else if(pid->mode == PID_MODE_POSITION_D_ON_MEASUREMENT)
    {
        pid->set = exp_data;

        pid->err3[2] = pid->err3[1];
        pid->err3[1] = pid->err3[0];
        pid->real3[2] = pid->real3[1];
        pid->real3[1] = pid->real3[0];
        pid->real3[0] = real_data;
        pid->err3[0] = exp_data - real_data;// Error calculation

        pid->pOut = pid->kp * pid->err3[0];
        pid->iOut += pid->ki * pid->err3[0];
        pid->dOut = -pid->kd * (pid->real3[0] - pid->real3[1]);

        if(pid->iOutMax != 0)// Apply integral limit
        {
            pid->iOut = pid_Data_Limit(pid->iOut, -(pid->iOutMax), pid->iOutMax);
        }

        pid->out = pid->pOut + pid->iOut + pid->dOut;

        if(pid->outMax != 0)// Apply output limit
        {
            pid->out = pid_Data_Limit(pid->out, -(pid->outMax), pid->outMax);
        }
    }
    else if(pid->mode == PID_MODE_ADD)
    {
        pid->set = exp_data;
        pid->err3[2] = pid->err3[1];
        pid->err3[1] = pid->err3[0];
        pid->err3[0] = exp_data - real_data;// Error calculation

        pid->pOut = pid->kp * (pid->err3[0] - pid->err3[1]);
        pid->iOut = pid->ki * pid->err3[0];           // Positional integral (integrate current error)
        pid->dOut = pid->kd * ((pid->err3[0]-pid->err3[1])-(pid->err3[1]-pid->err3[2]));

        pid->out += pid->pOut + pid->iOut + pid->dOut ;

        if(pid->outMax != 0)// Apply output limit
        {
            pid->out = pid_Data_Limit(pid->out, -(pid->outMax), pid->outMax);
        }
    }

    return pid->out;
}

/* Initialize PID handle with configuration parameters */
void PID_init(PID_HandleTypeDef *pid, PID_InitTypeDef *PID)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }

    pid->mode                    = PID->mode;// Positional, incremental, or d-on-measurement
    pid->kp                      = PID->kp;
    pid->ki                      = PID->ki;
    pid->kd                      = PID->kd;
    pid->iOutMax                 = PID->iOutMax;// Integral limit
    pid->outMax                  = PID->outMax;// Output limit
}

/* Clear all internal state of a PID handle (error history, outputs) */
void PID_cleardata(PID_HandleTypeDef *pid)
{
    pid->err3[0] = 0;
    pid->err3[1] = 0;
    pid->err3[2] = 0;
    pid->real3[0] = 0;
    pid->real3[1] = 0;
    pid->real3[2] = 0;
    pid->pOut = 0;
    pid->iOut = 0;
    pid->dOut = 0;
    pid->out = 0;
}
