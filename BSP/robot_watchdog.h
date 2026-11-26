/* 作者：Feng-100M
 * 文件名：robot_watchdog.h
 * 用途：声明 STM32H750 独立看门狗接口。
 * 日期：2026-11-25
 */
#ifndef ROBOT_WATCHDOG_H
#define ROBOT_WATCHDOG_H

#include <stdbool.h>

/* 调度前启动 IWDG1，按标称 32 kHz LSI 配置约 2 秒。
 * 等待配置同步最多 100 毫秒；失败返回 false，已启动的硬件会复位。
 */
bool robot_watchdog_start(void);

/* 电机任务确认通信任务仍在运行后调用。无阻塞，无返回值。 */
void robot_watchdog_refresh(void);

#endif /* ROBOT_WATCHDOG_H 结束 */
