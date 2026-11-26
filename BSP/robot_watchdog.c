/* 作者：Feng-100M
 * 文件名：robot_watchdog.c
 * 用途：依据 RM0433 的 IWDG_KR、PR 和 RLR 配置独立看门狗。
 * 日期：2026-11-25
 */
#include "./robot_watchdog.h"
#include "stm32h7xx_hal.h"

#define ROBOT_WATCHDOG_START (0xccccU)
#define ROBOT_WATCHDOG_UNLOCK (0x5555U)
#define ROBOT_WATCHDOG_REFRESH (0xaaaaU)
#define ROBOT_WATCHDOG_DIV32 (3U)
#define ROBOT_WATCHDOG_RELOAD (1999U)
#define ROBOT_WATCHDOG_SYNC_MS (100U)

bool robot_watchdog_start(void)
{
    uint32_t start_ms = HAL_GetTick();
    IWDG1->KR = ROBOT_WATCHDOG_START;
    IWDG1->KR = ROBOT_WATCHDOG_UNLOCK;
    IWDG1->PR = ROBOT_WATCHDOG_DIV32;
    IWDG1->RLR = ROBOT_WATCHDOG_RELOAD;
    while (IWDG1->SR != 0U)
    {
        if (HAL_GetTick() - start_ms >= ROBOT_WATCHDOG_SYNC_MS)
        {
            return false;
        }
    }
    robot_watchdog_refresh();
    return true;
}

void robot_watchdog_refresh(void)
{
    IWDG1->KR = ROBOT_WATCHDOG_REFRESH;
}
