/* 作者：Feng-100M
 * 文件名：robot_motor.h
 * 用途：四轮电机有符号 PWM 输出接口。
 * 日期：2026-11-25
 */
#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <stdbool.h>

/* 电机任务独占。四个有符号比较值按现有轮序排列，幅值限于定时器周期。
 * 传入空指针时关闭全部 PWM 并返回 false。零值表示零占空比。
 */
bool robot_motor_write(const int command[4]);

/* 致命故障停机专用。仅清零已供钟的 PWM 定时器，可在调度前或异常上下文调用。
 * 调用方须阻止电机任务继续运行。无等待，不依赖 HAL 句柄初始化进度。
 */
void robot_motor_emergency_stop(void);

#endif /* ROBOT_MOTOR_H 结束 */
