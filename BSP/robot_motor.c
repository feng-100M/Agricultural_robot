/* 作者：Feng-100M
 * 文件名：robot_motor.c
 * 用途：保留底盘四组方向引脚、PWM 通道和轮序。
 * 日期：2026-11-25
 */
#include <stddef.h>
#include <stdint.h>

#include "./robot_motor.h"
#include "../Core/Inc/tim.h"

typedef struct robot_motor_channel
{
    GPIO_TypeDef *port_a;
    uint16_t pin_a;
    GPIO_TypeDef *port_b;
    uint16_t pin_b;
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    void (*write)(const struct robot_motor_channel *, int);
} robot_motor_channel;

/* 按 command 的符号选择方向，幅值作为 PWM 比较值并限于 ARR + 1。
 * motor 指向板级接线表中的一路电机，仅由电机任务写入对应通道。
 */
static void robot_motor_write_pwm(const robot_motor_channel *motor, int command)
{
    /* 先转无符号再求幅值，可表示 INT_MIN 的绝对值。 */
    uint32_t magnitude = command < 0 ? 0U - (uint32_t)command : (uint32_t)command;
    /* TIM12/TIM15 的 ARR 为 99，比较值 100 表示原工程的满占空比。 */
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor->timer) + 1U;
    HAL_GPIO_WritePin(motor->port_a, motor->pin_a, command > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->port_b, motor->pin_b, command > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, magnitude > period ? period : magnitude);
}

/* 只读板级接线表，生命周期为整次上电。 */
static const robot_motor_channel g_robot_motor_channels[] = {
    {GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5, &htim12, TIM_CHANNEL_1, robot_motor_write_pwm},
    {GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, &htim12, TIM_CHANNEL_2, robot_motor_write_pwm},
    {GPIOB, GPIO_PIN_2, GPIOE, GPIO_PIN_7, &htim15, TIM_CHANNEL_1, robot_motor_write_pwm},
    {GPIOE, GPIO_PIN_8, GPIOE, GPIO_PIN_9, &htim15, TIM_CHANNEL_2, robot_motor_write_pwm}
};

/* PWM 比较值启用了预装载，写入后产生一次更新事件，使零占空比立即生效。 */
static void robot_motor_clear_pwm(TIM_TypeDef *timer)
{
    timer->CCR1 = 0U;
    timer->CCR2 = 0U;
    timer->EGR = TIM_EGR_UG;
}

void robot_motor_emergency_stop(void)
{
    if (__HAL_RCC_TIM12_IS_CLK_ENABLED())
    {
        robot_motor_clear_pwm(TIM12);
    }
    if (__HAL_RCC_TIM15_IS_CLK_ENABLED())
    {
        robot_motor_clear_pwm(TIM15);
    }
}

bool robot_motor_write(const int command[4])
{
    size_t index;
    for (index = 0U; index < sizeof(g_robot_motor_channels) / sizeof(g_robot_motor_channels[0]); ++index)
    {
        const robot_motor_channel *motor = &g_robot_motor_channels[index];
        motor->write(motor, command == NULL ? 0 : command[index]);
    }
    return command != NULL;
}
