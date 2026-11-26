/*
 * pid_1.h
 *
 *  Created on: Jun 1, 2024
 *      Author: Feng-100M
 */
#ifndef _PID_H
#define _PID_H

#include "main.h"


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA    = 1
};

typedef struct
{
    uint8_t mode;
    float kp;
    float ki;
    float kd;

    float max_out;
    float max_iout;

    float measure;
    float set;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];
}pid_type_def;

void pid_init(pid_type_def* pid,uint8_t mode,const float PID[3],float max_out,float max_iout);
void limit_max(float* num , float max);
float pid_calc(pid_type_def* pid , float measure , float set);
void pid_clean(pid_type_def* pid);

#endif


