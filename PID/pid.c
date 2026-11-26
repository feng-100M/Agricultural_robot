/*
 * pid_1.c
 *
 *  Created on: Jun 1, 2024
 *      Author: Feng-100M
 */
#include "pid.h"

/* 配置位置式或增量式 PID，PID 数组依次给出比例、积分和微分系数，两个限幅值对应总输出和积分项。
 * 清零误差和微分历史，保留已有的累计输出；首次使用应提供零初始化实例。任一指针为空时直接返回。
 */
void pid_init(pid_type_def* pid,uint8_t mode,const float PID[3],float max_out,float max_iout)
{
    if(pid == NULL || PID == NULL)
    {
        return;
    }
    pid->kp = PID[0];
    pid->ki = PID[1];
    pid->kd = PID[2];
    pid->mode = mode;
    pid->max_iout = max_iout;
    pid->max_out = max_out;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
}

/* 将 num 指向的值限制在正负 |max| 之间。max 为零时保留原值，num 须指向可写浮点数。 */
void limit_max(float* num , float max)
{
    if (max > 0)
    {
        if(*num >= max)
        {
            *num = max;
        }
        else if (*num <= -max)
        {
            *num = -max;
        }
    }
    else if (max < 0)
    {
        if(*num <= max)
        {
            *num = max;
        }
        else if (*num >= -max)
        {
            *num = -max;
        }
    }
}

/* 使用同单位的测量值 measure 和目标值 set 更新一个采样周期，返回该实例的控制输出。
 * 调用方按整定时的采样周期独占更新 pid；空指针返回 0，未知模式保留原输出。
 */
float pid_calc(pid_type_def* pid , float measure , float set)
{
    if(pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->measure = measure;
    pid->set = set;
    pid->error[1] = pid->error[0];
    pid->error[0] = set - measure;
    switch(pid->mode)
    {
        case PID_POSITION://位置PID/角度PID
            pid->Pout = pid->kp * pid->error[0];
            pid->Iout += pid->ki * pid->error[0];
//            pid->Dbuf[2] = pid->Dbuf[1];
//            pid->Dbuf[1] = pid->Dbuf[0];
            pid->Dbuf[0] = pid->error[0] - pid->error[1];
            pid->Dout = pid->kd * pid->Dbuf[0];
            limit_max(&pid->Iout, pid->max_iout);
            pid->out = pid->Pout + pid->Iout + pid->Dout;
            limit_max(&pid->out, pid->max_out);
            break;
        case PID_DELTA://速度PID
            pid->Pout = pid->kp * (pid->error[0] - pid->error[1]);
            pid->Iout = pid->ki * pid->error[0];
            pid->Dbuf[2] = pid->Dbuf[1];
            pid->Dbuf[1] = pid->Dbuf[0];
            pid->Dbuf[0] = pid->error[0] - 2.0f*pid->error[1] + pid->error[2];
            pid->Dout = pid->kd * pid->Dbuf[0];
            pid->out += pid->Pout + pid->Iout + pid->Dout;
            limit_max(&pid->out, pid->max_out);
            break;
    }
    return pid->out;
}

/* 清除 pid 的误差历史、测量值和累计输出，保留增益、限幅与模式。调用方须独占有效实例。 */
void pid_clean(pid_type_def* pid)
{
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->measure = pid->set = 0.0f;
}














