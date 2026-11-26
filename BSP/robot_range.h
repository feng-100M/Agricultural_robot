/* 作者：Feng-100M
 * 文件名：robot_range.h
 * 用途：获取同一轮前、后、左三个测距样本。
 * 日期：2026-11-25
 */
#ifndef ROBOT_RANGE_H
#define ROBOT_RANGE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t front_mm;
    uint16_t rear_mm;
    uint16_t left_mm;
} robot_range_sample;

/* 在 GPIO 和 TIM7 就绪后由测距任务调用。三路设备均改址并配置成功才返回 true。 */
bool robot_range_init(void);

/* 仅由测距任务在设备初始化后调用，阻塞时间由三次单次测量决定。
 * 成功时交付三个毫米值，任一设备或测距状态失败时返回 false 并保留输出。
 */
bool robot_range_read(robot_range_sample *sample);

#endif /* ROBOT_RANGE_H 结束 */
