/* 作者：Feng-100M
 * 文件名：robot_range.c
 * 用途：按原安装顺序读取 VL53L0X，过滤无效测距状态。
 * 日期：2026-11-25
 */
#include <stddef.h>

#include "./robot_range.h"
#include "../VL53L0X/demo/vl53l0x.h"
#include "../Core/Inc/main.h"

/* 对已初始化的 device 执行一次阻塞测距，distance_mm 指向调用方的可写结果。
 * 仅当通信成功且 RangeStatus 为零时写回毫米值并返回 true，失败时保留原值。
 */
static bool robot_range_read_device(VL53L0X_Dev_t *device, uint16_t *distance_mm)
{
    VL53L0X_RangingMeasurementData_t result;
    if (VL53L0X_PerformSingleRangingMeasurement(device, &result) != VL53L0X_ERROR_NONE ||
        result.RangeStatus != 0U)
    {
        return false;
    }
    *distance_mm = result.RangeMilliMeter;
    return true;
}

bool robot_range_init(void)
{
    return vl53l0x_init_All() == VL53L0X_ERROR_NONE;
}

bool robot_range_read(robot_range_sample *sample)
{
    robot_range_sample next;
    if (sample == NULL ||
        !robot_range_read_device(&vl53l0x_dev, &next.front_mm) ||
        !robot_range_read_device(&vl53l0x_dev1, &next.rear_mm) ||
        !robot_range_read_device(&vl53l0x_dev2, &next.left_mm))
    {
        return false;
    }
    *sample = next;
    return true;
}
