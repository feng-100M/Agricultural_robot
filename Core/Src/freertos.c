/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../../BSP/robot_motor.h"
#include "../../BSP/robot_range.h"
#include "../../BSP/robot_uart.h"
#include "../../BSP/robot_watchdog.h"
#include "../../MPU6050/MPU6050.h"
#include "../../PID/pid.h"
#include "../../PS2/ps2.h"
#include "../../modules/robot_link.h"
#include "event_groups.h"
#include "queue.h"
#include "tim.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint32_t progress_ms;
    uint32_t heartbeat_ms;
    uint32_t legacy_request_ms;
    uint8_t spray_request;
    bool supervised;
    bool stopped;
} robot_control_status;

/* 同一指令编号在手动和自动模式下沿用原有含义。 */
typedef enum
{
    /* 上电尚未选择动作。 */
    ROBOT_MOTION_UNSET = 0,
    /* 向前行驶。 */
    ROBOT_MOTION_FORWARD = 1,
    /* 向后行驶。 */
    ROBOT_MOTION_REVERSE = 2,
    /* 自动前端减速，手动向右平移。 */
    ROBOT_MOTION_FRONT_APPROACH = 3,
    /* 自动后端减速，手动向左平移。 */
    ROBOT_MOTION_REAR_APPROACH = 4,
    /* 自动向左换行，手动正向旋转。 */
    ROBOT_MOTION_SHIFT_LEFT = 5,
    /* 自动向左减速。 */
    ROBOT_MOTION_SLOW_LEFT = 6,
    /* 自动按前距右移，手动反向旋转。 */
    ROBOT_MOTION_FRONT_SHIFT = 7,
    /* 自动按后距右移。 */
    ROBOT_MOTION_REAR_SHIFT = 8,
    /* 末行向右退出。 */
    ROBOT_MOTION_EXIT_SHIFT = 9,
    /* 目标速度归零。 */
    ROBOT_MOTION_STOP = 10,
    /* 起步向左移入首行。 */
    ROBOT_MOTION_ENTRY = 11
} robot_motion_command;

typedef enum
{
    /* 尚未进垄，或行程已结束。 */
    ROBOT_ROUTE_IDLE,
    /* 向后行驶，等待后端换垄。 */
    ROBOT_ROUTE_REVERSE,
    /* 向前行驶，等待前端换垄。 */
    ROBOT_ROUTE_FORWARD,
    /* 最后一次换垄后退出。 */
    ROBOT_ROUTE_EXIT
} robot_route_phase;

typedef struct
{
    float forward;
    float lateral;
    float rotation;
} robot_velocity;

typedef struct
{
    int yaw;
    int front;
    int rear;
    int left;
} robot_motion_trim;

typedef struct
{
    robot_range_sample sample;
    uint32_t started_ms;
    bool valid;
} robot_range_update;

typedef struct
{
    bool range_valid;
    bool front;
    bool rear;
    bool left;
    robot_range_sample range;
    float reference_rpm;
} robot_route_input;

typedef struct
{
    robot_route_phase phase;
    bool entered;
    bool counting_shift;
    bool counting_exit;
    bool slow_exit_pending;
    unsigned int completed_rows;
    double entry_m;
    double shift_m;
    double exit_m;
} robot_route_state;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROBOT_CONTROL_PROGRESS_MS (250U)
#define ROBOT_CONTROL_WAKE_MS (20U)
#define ROBOT_CONTROL_TX_TIMEOUT_MS (5U)
#define ROBOT_MOTION_PERIOD_MS (10U)
#define ROBOT_PS2_PERIOD_MS (15U)
#define ROBOT_IMU_PERIOD_MS (30U)
/* 三次默认单次测距的预算各为 33 毫秒，另留总线和任务调度时间。 */
#define ROBOT_RANGE_MAX_AGE_MS (500U)
#define ROBOT_RANGE_INIT_RETRY_MS (1000U)
#define ROBOT_IMU_SAMPLE_EVENT (1U << 0)
#define ROBOT_PS2_ANALOG_MODE (0x73U)
#define ROBOT_PS2_START_BUTTON (0x04U)
#define ROBOT_PS2_VERTICAL_CENTER (127U)
#define ROBOT_PS2_HORIZONTAL_CENTER (128U)
#define ROBOT_FRONT_REAR_TARGET_MM (100U)
#define ROBOT_LEFT_TARGET_MM (80U)
#define ROBOT_LEFT_LIMIT_MM (200U)
#define ROBOT_ENTRY_DISTANCE_MM (175U)
#define ROBOT_ROW_END_MM (150U)
#define ROBOT_EXIT_DISTANCE_MM (200U)
#define ROBOT_ROW_COUNT (3U)
#define ROBOT_ENTRY_TRAVEL_M (0.50)
#define ROBOT_SHIFT_TRAVEL_M (0.55)
#define ROBOT_EXIT_TRAVEL_M (0.50)
#define ROBOT_WHEEL_DIAMETER_M (0.08)
#define ROBOT_ENCODER_COUNTS_PER_REV (11.0 * 30.0 * 4.0)
/* 保留底盘实测使用的原换算系数和圆周率近似值。 */
#define ROBOT_ENCODER_RPM_SCALE (2000.0)
#define ROBOT_WHEEL_PI (3.14)
#define ROBOT_ROTATION_ARM (0.2F)
#define ROBOT_CONTROL_BATCH_BYTES (32U)
#define ROBOT_PUMP_ON (1U)
#define ROBOT_PUMP_OFF (2U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 调度前初始化，整个上电周期由电机任务独占。四轮顺序与 PWM 输出 A、B、C、D 一致。 */
static pid_type_def g_robot_wheel_pid[4];
static pid_type_def g_robot_yaw_pid;
/* 前后测距依次使用同一控制器，共享误差历史；左侧测距使用独立控制器。 */
static pid_type_def g_robot_front_rear_range_pid;
static pid_type_def g_robot_left_range_pid;

/* 单槽快照只保留最新值。通信任务 -> control_queue -> 电机任务（窥读）；
 * 测距任务 -> range_queue -> 电机任务（取走）。两条队列的存储持续整个上电周期。
 */
static StaticQueue_t g_robot_control_queue_storage;
static uint8_t g_robot_control_slot[sizeof(robot_control_status)];
static QueueHandle_t g_robot_control_queue;
static StaticQueue_t g_robot_range_queue_storage;
static uint8_t g_robot_range_slot[sizeof(robot_range_update)];
static QueueHandle_t g_robot_range_queue;
static robot_control_status g_robot_control_status;
static robot_link_receiver g_robot_link_receiver;

/* 以下状态在本次上电期间由任务共享。发布手柄、控制指令和传感器快照时保护一致性。 */
static robot_motion_command g_robot_motion_command;
static robot_route_input g_robot_route_input;
static bool g_robot_route_enabled;
static bool g_robot_entry_counting;

/* 保留原有实际施药请求的外部观察点，供板上调试读取。 */
uint8_t g_robot_spray_request;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Control_Task */
osThreadId_t Control_TaskHandle;
const osThreadAttr_t Control_Task_attributes = {
  .name = "Control_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor_Task */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Tof_F_Task */
osThreadId_t Tof_F_TaskHandle;
const osThreadAttr_t Tof_F_Task_attributes = {
  .name = "Tof_F_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PS2Task */
osThreadId_t PS2TaskHandle;
const osThreadAttr_t PS2Task_attributes = {
  .name = "PS2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Type_Task */
osThreadId_t Type_TaskHandle;
const osThreadAttr_t Type_Task_attributes = {
  .name = "Type_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for YawData */
osMessageQueueId_t YawDataHandle;
const osMessageQueueAttr_t YawData_attributes = {
  .name = "YawData"
};
/* Definitions for MPU6050 */
osTimerId_t MPU6050Handle;
const osTimerAttr_t MPU6050_attributes = {
  .name = "MPU6050"
};
/* Definitions for ControlEvent */
osEventFlagsId_t ControlEventHandle;
const osEventFlagsAttr_t ControlEvent_attributes = {
  .name = "ControlEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void robot_control_deliver(const robot_link_event *event);
static bool robot_control_allow_motion(void);
static uint8_t robot_control_pump_request(const robot_control_status *status, uint32_t now_ms);
static robot_motion_command robot_ps2_decode(const JOYSTICK_TypeDef *joystick);
static void robot_route_step(robot_route_state *route, const robot_route_input *input);
static void robot_motion_select(bool manual, robot_motion_command command,
                                const robot_motion_trim *trim, robot_velocity *velocity);
static void robot_motion_drive(const robot_velocity *velocity, float wheel_rpm[4]);
static void robot_motion_write(const int pwm[4]);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControl(void *argument);
void StartMotor(void *argument);
void StartTof_F(void *argument);
void StartPS2(void *argument);
void StartType(void *argument);
void MPU6050_Callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
/* FreeRTOS 已启用空闲钩子。此处直接返回，供内核继续回收已删除任务。 */
void vApplicationIdleHook(void)
{
}
/* USER CODE END 2 */

/* 由 main 在调度开始前调用，配置 PID、任务、单槽队列、事件组和姿态采样定时器。
 * 检查对象句柄后启动 IWDG1，创建对象或启动看门狗失败时进入 Error_Handler。
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    const float wheel_gains[3] = {5.0F, 0.2F, 0.0F};
    const float yaw_gains[3] = {2.5F, 0.0F, 0.3F};
    const float front_rear_gains[3] = {0.15F, 0.0F, 0.005F};
    const float left_gains[3] = {0.5F, 0.0F, 0.02F};
    size_t wheel;

    for (wheel = 0U; wheel < sizeof(g_robot_wheel_pid) / sizeof(g_robot_wheel_pid[0]); ++wheel)
    {
        pid_init(&g_robot_wheel_pid[wheel], PID_DELTA, wheel_gains, 100.0F, 5.0F);
    }
    pid_init(&g_robot_yaw_pid, PID_POSITION, yaw_gains, 500.0F, 5.0F);
    pid_init(&g_robot_front_rear_range_pid, PID_POSITION, front_rear_gains, 100.0F, 5.0F);
    pid_init(&g_robot_left_range_pid, PID_POSITION, left_gains, 100.0F, 5.0F);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of MPU6050 */
  MPU6050Handle = osTimerNew(MPU6050_Callback, osTimerPeriodic, NULL, &MPU6050_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of YawData */
  YawDataHandle = osMessageQueueNew (1, sizeof(float), &YawData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    g_robot_control_queue = xQueueCreateStatic(1U, sizeof(robot_control_status),
                                                g_robot_control_slot, &g_robot_control_queue_storage);
    g_robot_range_queue = xQueueCreateStatic(1U, sizeof(robot_range_update),
                                              g_robot_range_slot, &g_robot_range_queue_storage);
    if (g_robot_control_queue == NULL || g_robot_range_queue == NULL)
    {
        Error_Handler();
    }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Control_Task */
  Control_TaskHandle = osThreadNew(StartControl, NULL, &Control_Task_attributes);

  /* creation of Motor_Task */
  Motor_TaskHandle = osThreadNew(StartMotor, NULL, &Motor_Task_attributes);

  /* creation of Tof_F_Task */
  Tof_F_TaskHandle = osThreadNew(StartTof_F, NULL, &Tof_F_Task_attributes);

  /* creation of PS2Task */
  PS2TaskHandle = osThreadNew(StartPS2, NULL, &PS2Task_attributes);

  /* creation of Type_Task */
  Type_TaskHandle = osThreadNew(StartType, NULL, &Type_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of ControlEvent */
  ControlEventHandle = osEventFlagsNew(&ControlEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
    if (defaultTaskHandle == NULL || Motor_TaskHandle == NULL || Control_TaskHandle == NULL ||
        Type_TaskHandle == NULL || Tof_F_TaskHandle == NULL || PS2TaskHandle == NULL ||
        ControlEventHandle == NULL || YawDataHandle == NULL || MPU6050Handle == NULL)
    {
        Error_Handler();
    }
    if (!robot_watchdog_start())
    {
        Error_Handler();
    }
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/* 通信任务独占 USART2 接收器，按字节解析并向电机任务发布最新控制状态。
 * 心跳超时、启动接收失败或字节流中断会锁存停车请求；恢复 DMA 后仍保留该请求。
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    uint8_t bytes[ROBOT_CONTROL_BATCH_BYTES];
    bool running = false;
    (void)argument;
    for (;;)
    {
        bool fault = false;
        size_t count;
        size_t index;
        if (!running)
        {
            running = robot_uart_start();
            if (!running)
            {
                g_robot_control_status.stopped = true;
            }
        }
        count = running ? robot_uart_read(bytes, sizeof(bytes), &fault) : 0U;
        if (fault)
        {
            g_robot_control_status.stopped = true;
            g_robot_link_receiver.used = 0U;
            g_robot_link_receiver.phase = ROBOT_LINK_DROP_LINE;
            running = false;
        }
        for (index = 0U; index < count; ++index)
        {
            /* 上一帧的 ACK 可能阻塞，解析器只在处理行尾时使用毫秒时钟。 */
            uint32_t frame_ms = bytes[index] == '\n' ? HAL_GetTick() : 0U;
            robot_link_feed(&g_robot_link_receiver, bytes[index], frame_ms, robot_control_deliver);
        }
        g_robot_control_status.progress_ms = HAL_GetTick();
        if (robot_link_expired(&g_robot_link_receiver, g_robot_control_status.progress_ms))
        {
            g_robot_control_status.stopped = true;
        }
        g_robot_control_status.spray_request =
            robot_control_pump_request(&g_robot_control_status, g_robot_control_status.progress_ms);
        if (xQueueOverwrite(g_robot_control_queue, &g_robot_control_status) != pdPASS)
        {
            Error_Handler();
        }
        if (count == 0U)
        {
            /* 定时唤醒用于监督会话和恢复 DMA，返回零表示当前无通知。 */
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ROBOT_CONTROL_WAKE_MS));
        }
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControl */
/* 姿态任务按软件定时器事件读取 DMP，将有限的偏航角（度）覆盖写入单槽队列。
 * FIFO 读取失败或未产生有效偏航角时跳过本次发布。
 */
/* USER CODE END Header_StartControl */
void StartControl(void *argument)
{
  /* USER CODE BEGIN StartControl */
    (void)argument;
    if (osTimerStart(MPU6050Handle, pdMS_TO_TICKS(ROBOT_IMU_PERIOD_MS)) != osOK)
    {
        Error_Handler();
    }
    for (;;)
    {
        float pitch_deg;
        float roll_deg;
        /* 驱动在 FIFO 未携带四元数时也可能返回成功，过滤未写入的偏航角。 */
        float yaw_deg = NAN;
        EventBits_t events = xEventGroupWaitBits(ControlEventHandle, ROBOT_IMU_SAMPLE_EVENT,
                                                  pdTRUE, pdFALSE, portMAX_DELAY);
        if ((events & ROBOT_IMU_SAMPLE_EVENT) != 0U &&
            MPU6050_DMP_Get_Date(&pitch_deg, &roll_deg, &yaw_deg) == 0 && isfinite(yaw_deg))
        {
            if (xQueueOverwrite(YawDataHandle, &yaw_deg) != pdPASS)
            {
                Error_Handler();
            }
        }
    }
  /* USER CODE END StartControl */
}

/* USER CODE BEGIN Header_StartMotor */
/* 电机任务以 10 毫秒延时分隔控制周期，根据手动或导航指令更新四轮闭环输出。
 * 通信故障时关闭 PWM 和 PD8；自动导航还须有新鲜测距。仅收到有效开启请求时拉高 PD8。
 */
/* USER CODE END Header_StartMotor */
void StartMotor(void *argument)
{
  /* USER CODE BEGIN StartMotor */
    robot_velocity velocity = {0};
    robot_motion_trim trim = {0};
    robot_route_input input = {0};
    robot_range_update update = {0};
    const int zero_pwm[4] = {0};
    (void)argument;
    for (;;)
    {
        robot_motion_command command;
        bool manual;
        bool received;
        bool range_stop;
        float yaw_deg;
        float wheel_rpm[4];
        vTaskDelay(pdMS_TO_TICKS(ROBOT_MOTION_PERIOD_MS));
        if (!robot_control_allow_motion())
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
            robot_motion_write(zero_pwm);
            taskENTER_CRITICAL();
            g_robot_route_input.range_valid = false;
            taskEXIT_CRITICAL();
            continue;
        }

        taskENTER_CRITICAL();
        manual = my_joystick.mode == ROBOT_PS2_ANALOG_MODE;
        if (my_joystick.btn2 == ROBOT_PS2_START_BUTTON)
        {
            g_robot_route_enabled = true;
            g_robot_entry_counting = true;
            g_robot_motion_command = ROBOT_MOTION_ENTRY;
        }
        command = g_robot_motion_command;
        taskEXIT_CRITICAL();

        /* 三路光电均为低有效，后端实际接在 PA10。 */
        input.front = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET;
        input.left = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET;
        input.rear = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET;

        received = xQueueReceive(g_robot_range_queue, &update, 0U) == pdPASS;
        if (HAL_GetTick() - update.started_ms >= ROBOT_RANGE_MAX_AGE_MS)
        {
            update.valid = false;
        }
        input.range_valid = update.valid;
        range_stop = !manual && !input.range_valid;
        /* 本周期沿用上一次的纠偏量。新样本只影响下一周期的目标轮速。 */
        if (!range_stop)
        {
            robot_motion_select(manual, command, &trim, &velocity);
        }
        if (xQueueReceive(YawDataHandle, &yaw_deg, 0U) == pdPASS)
        {
            trim.yaw = (int)pid_calc(&g_robot_yaw_pid, yaw_deg, 0.0F);
        }
        if (received && input.range_valid)
        {
            input.range = update.sample;
            /* 前后向共用同一 PID 实例，按原有顺序更新其历史状态。 */
            trim.front = (int)pid_calc(&g_robot_front_rear_range_pid, input.range.front_mm, ROBOT_FRONT_REAR_TARGET_MM);
            trim.rear = (int)pid_calc(&g_robot_front_rear_range_pid, input.range.rear_mm, ROBOT_FRONT_REAR_TARGET_MM);
            trim.left = (int)pid_calc(&g_robot_left_range_pid, input.range.left_mm, ROBOT_LEFT_TARGET_MM);
        }
        if (!input.left || input.range.left_mm > ROBOT_LEFT_LIMIT_MM)
        {
            trim.left = 0;
        }

        /* PD8 沿用原停车电平，低为关。自动作业失去测距时也撤销泵输出。 */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,
                          g_robot_spray_request == ROBOT_PUMP_ON && !(range_stop && g_robot_route_enabled)
                          ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (range_stop)
        {
            velocity = (robot_velocity){0};
            trim.front = trim.rear = trim.left = 0;
        }
        robot_motion_drive(range_stop ? NULL : &velocity, wheel_rpm);
        /* D 轮为原里程参考轮。整组发布，换垄任务每次读取同一周期的反馈。 */
        input.reference_rpm = wheel_rpm[3];
        taskENTER_CRITICAL();
        g_robot_route_input = input;
        taskEXIT_CRITICAL();
    }
  /* USER CODE END StartMotor */
}

/* USER CODE BEGIN Header_StartTof_F */
/* 测距任务独占三路器件，初始化失败时延时重试。每轮发布三路距离或无效状态，
 * 时间戳取第一路测量前的毫秒时钟，使最早取得的距离也受有效期约束。
 */
/* USER CODE END Header_StartTof_F */
void StartTof_F(void *argument)
{
  /* USER CODE BEGIN StartTof_F */
    robot_range_update update = {0};
    (void)argument;
    while (!robot_range_init())
    {
        vTaskDelay(pdMS_TO_TICKS(ROBOT_RANGE_INIT_RETRY_MS));
    }
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(ROBOT_MOTION_PERIOD_MS));
        update.started_ms = HAL_GetTick();
        update.valid = robot_range_read(&update.sample);
        if (xQueueOverwrite(g_robot_range_queue, &update) != pdPASS)
        {
            Error_Handler();
        }
    }
  /* USER CODE END StartTof_F */
}

/* USER CODE BEGIN Header_StartPS2 */
/* 手柄任务每轮延时 15 毫秒后读取一帧，在同一临界区内发布手柄快照。
 * 仅在模拟模式下更新手动运动指令，其他模式保留当前指令。
 */
/* USER CODE END Header_StartPS2 */
void StartPS2(void *argument)
{
  /* USER CODE BEGIN StartPS2 */
    JOYSTICK_TypeDef sample;
    (void)argument;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(ROBOT_PS2_PERIOD_MS));
        AX_PS2_ScanKey(&sample);
        taskENTER_CRITICAL();
        my_joystick = sample;
        if (sample.mode == ROBOT_PS2_ANALOG_MODE)
        {
            g_robot_motion_command = robot_ps2_decode(&sample);
        }
        taskEXIT_CRITICAL();
    }
  /* USER CODE END StartPS2 */
}

/* USER CODE BEGIN Header_StartType */
/* 导航任务持有跨周期的垄道状态，仅在路线已启用且手柄处于非模拟模式时推进。
 * 暂停任务调度期间读取电机任务的快照并更新运动指令，中断仍可执行。
 */
/* USER CODE END Header_StartType */
void StartType(void *argument)
{
  /* USER CODE BEGIN StartType */
    robot_route_state route = {0};
    (void)argument;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(ROBOT_MOTION_PERIOD_MS));
        /* 仅任务访问导航状态。暂停任务切换以完成一次状态转移，外设中断仍可运行。 */
        vTaskSuspendAll();
        if (my_joystick.mode != ROBOT_PS2_ANALOG_MODE && g_robot_route_enabled &&
            g_robot_route_input.range_valid)
        {
            robot_route_step(&route, &g_robot_route_input);
        }
        (void)xTaskResumeAll();
    }
  /* USER CODE END StartType */
}

/* FreeRTOS 软件定时器任务置位姿态采样事件。事件按位传递，重复置位可能合并为一次读取。 */
void MPU6050_Callback(void *argument)
{
  /* USER CODE BEGIN MPU6050_Callback */
    (void)argument;
    /* 回调在软件定时器任务中执行，姿态任务读取后清除此位。 */
    (void)xEventGroupSetBits(ControlEventHandle, ROBOT_IMU_SAMPLE_EVENT);
  /* USER CODE END MPU6050_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 在通信任务内同步处理单帧，event 仅在本次回调期间有效。停车已锁存时直接返回。
 * 新帧更新施药请求和心跳状态，重复帧只重发确认；旧格式指令不发送确认。
 */
static void robot_control_deliver(const robot_link_event *event)
{
    uint8_t ack[ROBOT_LINK_ACK_SIZE];
    if (g_robot_control_status.stopped)
    {
        return;
    }
    if (!event->duplicate)
    {
        /* 新会话从关泵开始，旧协议的开启请求不能由新会话的心跳续期。 */
        if (!g_robot_control_status.supervised && g_robot_link_receiver.active)
        {
            g_robot_control_status.spray_request = ROBOT_PUMP_OFF;
        }
        g_robot_control_status.supervised = g_robot_link_receiver.active;
        g_robot_control_status.heartbeat_ms = g_robot_link_receiver.heartbeat_ms;
        if (event->has_spray)
        {
            g_robot_control_status.spray_request = event->spray_on ? ROBOT_PUMP_ON : ROBOT_PUMP_OFF;
            if (event->legacy)
            {
                g_robot_control_status.legacy_request_ms = HAL_GetTick();
            }
        }
    }
    if (!event->legacy && robot_link_ack(event->sequence, ack))
    {
        /* 发送失败时保留序号，发送方重传后可再次确认；不重复执行业务。 */
        if (HAL_UART_Transmit(&huart2, ack, sizeof(ack), ROBOT_CONTROL_TX_TIMEOUT_MS) != HAL_OK)
        {
            ++g_robot_link_receiver.errors;
        }
    }
}

/* 发布快照和使用快照时均校验旧协议开泵期限。通信任务尚未发布下一次状态时，
 * 电机任务也能在自己的控制周期撤销过期开泵请求。毫秒差值允许时钟回绕。
 */
static uint8_t robot_control_pump_request(const robot_control_status *status, uint32_t now_ms)
{
    if (!status->supervised && status->spray_request == ROBOT_PUMP_ON &&
        now_ms - status->legacy_request_ms >= ROBOT_LINK_TIMEOUT_MS)
    {
        return ROBOT_PUMP_OFF;
    }
    return status->spray_request;
}

/* 电机任务读取最近的通信状态，无状态时返回 false。
 * 通信进度正常时喂狗；进度、会话心跳超时或收到停车状态时锁存停车，直至复位。
 */
static bool robot_control_allow_motion(void)
{
    static bool stopped;
    robot_control_status status;
    uint32_t now_ms;
    if (xQueuePeek(g_robot_control_queue, &status, 0U) != pdPASS)
    {
        return false;
    }
    now_ms = HAL_GetTick();
    if (now_ms - status.progress_ms >= ROBOT_CONTROL_PROGRESS_MS)
    {
        stopped = true;
    }
    else
    {
        robot_watchdog_refresh();
    }
    if (status.stopped || (status.supervised &&
        now_ms - status.heartbeat_ms >= ROBOT_LINK_TIMEOUT_MS))
    {
        stopped = true;
    }
    g_robot_spray_request = stopped ? ROBOT_PUMP_OFF : robot_control_pump_request(&status, now_ms);
    return !stopped;
}

/* 摇杆同时偏移时，旋转优先于横移，横移优先于前后移动。 */
static robot_motion_command robot_ps2_decode(const JOYSTICK_TypeDef *joystick)
{
    if (joystick->RJoy_LR != ROBOT_PS2_HORIZONTAL_CENTER)
    {
        return joystick->RJoy_LR > ROBOT_PS2_HORIZONTAL_CENTER ? ROBOT_MOTION_SHIFT_LEFT : ROBOT_MOTION_FRONT_SHIFT;
    }
    if (joystick->LJoy_LR != ROBOT_PS2_HORIZONTAL_CENTER)
    {
        return joystick->LJoy_LR > ROBOT_PS2_HORIZONTAL_CENTER ? ROBOT_MOTION_FRONT_APPROACH : ROBOT_MOTION_REAR_APPROACH;
    }
    if (joystick->LJoy_UD != ROBOT_PS2_VERTICAL_CENTER)
    {
        return joystick->LJoy_UD < ROBOT_PS2_VERTICAL_CENTER ? ROBOT_MOTION_FORWARD : ROBOT_MOTION_REVERSE;
    }
    return ROBOT_MOTION_STOP;
}

/* 按原轮径及采样标定，将参考轮转速折算为单周期里程。边界采用严格大于。 */
static bool robot_route_passed(bool counting, float reference_rpm, double *total_m, double limit_m)
{
    if (!counting)
    {
        return false;
    }
    float step_m = (float)(fabs(reference_rpm) * ROBOT_WHEEL_PI * ROBOT_WHEEL_DIAMETER_M /
                           ROBOT_ENCODER_RPM_SCALE);
    *total_m += step_m;
    return *total_m > limit_m;
}

/* 在任务调度锁内用本周期光电、距离（毫米）和参考轮转速推进进垄、换垄或退出状态。
 * route 由导航任务跨周期持有。调用方仅在本轮快照含有效测距时推进导航。
 * 换垄改变行进方向时同步更新 PD15，前进置位、后退复位。
 */
static void robot_route_step(robot_route_state *route, const robot_route_input *input)
{
    if (!route->entered && input->left)
    {
        if (input->range.left_mm <= ROBOT_ENTRY_DISTANCE_MM)
        {
            g_robot_motion_command = ROBOT_MOTION_FORWARD;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
            route->entered = true;
            route->phase = ROBOT_ROUTE_FORWARD;
        }
        else
        {
            g_robot_motion_command = ROBOT_MOTION_SLOW_LEFT;
        }
    }
    else if (robot_route_passed(g_robot_entry_counting, input->reference_rpm,
                                &route->entry_m, ROBOT_ENTRY_TRAVEL_M) && !route->entered)
    {
        g_robot_motion_command = ROBOT_MOTION_SHIFT_LEFT;
        route->entry_m = 0.0;
        g_robot_entry_counting = false;
    }

    if (route->phase == ROBOT_ROUTE_EXIT)
    {
        if (input->rear && input->range.rear_mm <= ROBOT_EXIT_DISTANCE_MM)
        {
            g_robot_motion_command = ROBOT_MOTION_EXIT_SHIFT;
            route->counting_exit = true;
        }
        else if (input->rear && route->slow_exit_pending)
        {
            g_robot_motion_command = ROBOT_MOTION_REAR_APPROACH;
            route->slow_exit_pending = false;
        }
        if (robot_route_passed(route->counting_exit, input->reference_rpm,
                               &route->exit_m, ROBOT_EXIT_TRAVEL_M) && !input->rear)
        {
            route->counting_exit = false;
            route->exit_m = 0.0;
            g_robot_motion_command = ROBOT_MOTION_STOP;
            route->phase = ROBOT_ROUTE_IDLE;
        }
        return;
    }
    if (route->phase == ROBOT_ROUTE_IDLE)
    {
        return;
    }

    /* 前后两端使用相同换垄距离，但检测端、目标动作与方向输出相反。 */
    bool forward = route->phase == ROBOT_ROUTE_FORWARD;
    bool approaching = forward ? input->front : input->rear;
    bool opposite_clear = forward ? !input->rear : !input->front;
    uint16_t distance_mm = forward ? input->range.front_mm : input->range.rear_mm;
    if (approaching)
    {
        if (distance_mm <= ROBOT_ROW_END_MM)
        {
            g_robot_motion_command = forward ? ROBOT_MOTION_FRONT_SHIFT : ROBOT_MOTION_REAR_SHIFT;
            route->counting_shift = true;
        }
        else
        {
            g_robot_motion_command = forward ? ROBOT_MOTION_FRONT_APPROACH : ROBOT_MOTION_REAR_APPROACH;
        }
    }
    if (robot_route_passed(route->counting_shift, input->reference_rpm,
                           &route->shift_m, ROBOT_SHIFT_TRAVEL_M) && opposite_clear)
    {
        route->shift_m = 0.0;
        route->counting_shift = false;
        g_robot_motion_command = forward ? ROBOT_MOTION_REVERSE : ROBOT_MOTION_FORWARD;
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
        route->phase = forward ? ROBOT_ROUTE_REVERSE : ROBOT_ROUTE_FORWARD;
        if (forward && ++route->completed_rows >= ROBOT_ROW_COUNT)
        {
            route->phase = ROBOT_ROUTE_EXIT;
            route->slow_exit_pending = true;
        }
    }
}

/* 基于原有标定选择前进、横移和旋转分量。未定义的模式组合沿用上周期目标。 */
static void robot_motion_select(bool manual, robot_motion_command command,
                                const robot_motion_trim *trim, robot_velocity *velocity)
{
    if (manual)
    {
        switch (command)
        {
            case ROBOT_MOTION_FORWARD:
                *velocity = (robot_velocity){70.0F, 0.0F, 0.0F};
                break;
            case ROBOT_MOTION_REVERSE:
                *velocity = (robot_velocity){-70.0F, 0.0F, 0.0F};
                break;
            case ROBOT_MOTION_FRONT_APPROACH:
                *velocity = (robot_velocity){0.0F, -35.0F, 0.0F};
                break;
            case ROBOT_MOTION_REAR_APPROACH:
                *velocity = (robot_velocity){0.0F, 60.0F, 0.0F};
                break;
            case ROBOT_MOTION_SHIFT_LEFT:
                *velocity = (robot_velocity){0.0F, 0.0F, 250.0F};
                break;
            case ROBOT_MOTION_FRONT_SHIFT:
                *velocity = (robot_velocity){0.0F, 0.0F, -250.0F};
                break;
            case ROBOT_MOTION_STOP:
                *velocity = (robot_velocity){0};
                break;
            default:
                break;
        }
        return;
    }
    switch (command)
    {
        case ROBOT_MOTION_FORWARD:
            *velocity = (robot_velocity){70.0F, -trim->left, trim->yaw};
            break;
        case ROBOT_MOTION_REVERSE:
            *velocity = (robot_velocity){-70.0F, -trim->left, trim->yaw};
            break;
        case ROBOT_MOTION_FRONT_APPROACH:
            *velocity = (robot_velocity){-trim->front, -trim->left, trim->yaw};
            break;
        case ROBOT_MOTION_REAR_APPROACH:
            *velocity = (robot_velocity){trim->rear, -trim->left, trim->yaw};
            break;
        case ROBOT_MOTION_SHIFT_LEFT:
            *velocity = (robot_velocity){trim->rear, 70.0F, trim->yaw * 3};
            break;
        case ROBOT_MOTION_SLOW_LEFT:
            *velocity = (robot_velocity){trim->rear, 40.0F, trim->yaw * 3};
            break;
        case ROBOT_MOTION_FRONT_SHIFT:
            *velocity = (robot_velocity){-trim->front, -40.0F, trim->yaw};
            break;
        case ROBOT_MOTION_REAR_SHIFT:
            *velocity = (robot_velocity){trim->rear, -35.0F, trim->yaw};
            break;
        case ROBOT_MOTION_EXIT_SHIFT:
            *velocity = (robot_velocity){0.0F, -35.0F, trim->yaw};
            break;
        case ROBOT_MOTION_STOP:
            *velocity = (robot_velocity){0};
            break;
        case ROBOT_MOTION_ENTRY:
            *velocity = (robot_velocity){0.0F, 60.0F, (float)(trim->yaw * 0.8)};
            break;
        default:
            break;
    }
}

/* 仅电机任务写 PWM。驱动拒绝指令时撤销水泵输出。 */
static void robot_motion_write(const int pwm[4])
{
    if (!robot_motor_write(pwm))
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
    }
}

/* 电机任务将底盘目标分解为四轮目标，并用本周期编码器计数计算 PID 输出。
 * velocity 为空表示禁止驱动，此时清零 PWM、轮速 PID 和编码器计数。
 * wheel_rpm 须容纳 A、B、C、D 四个反馈转速（转/分）。沿用底盘标定系数，B、D 反向换算，读取后清零计数器。
 */
static void robot_motion_drive(const robot_velocity *velocity, float wheel_rpm[4])
{
    TIM_TypeDef *const timers[] = {TIM4, TIM3, TIM5, TIM2};
    float targets[4];
    int pwm[4] = {0};
    size_t index;
    /* 暂停自动运动时仍清空编码器增量，避免恢复后的首周期累积整段停车时间。 */
    if (velocity == NULL)
    {
        robot_motion_write(pwm);
        for (index = 0U; index < 4U; ++index)
        {
            timers[index]->CNT = 0U;
            wheel_rpm[index] = 0.0F;
            pid_clean(&g_robot_wheel_pid[index]);
        }
        return;
    }
    const float forward = velocity->forward;
    const float lateral = velocity->lateral;
    const float turn = velocity->rotation * ROBOT_ROTATION_ARM;
    targets[0] = forward - lateral - turn;
    targets[1] = forward + lateral + turn;
    targets[2] = forward + lateral - turn;
    targets[3] = forward - lateral + turn;
    for (index = 0U; index < 4U; ++index)
    {
        int16_t counts = (int16_t)timers[index]->CNT;
        double rpm = counts / ROBOT_ENCODER_COUNTS_PER_REV * ROBOT_ENCODER_RPM_SCALE;
        wheel_rpm[index] = (float)((index == 1U || index == 3U) ? -rpm : rpm);
    }
    for (index = 0U; index < 4U; ++index)
    {
        timers[index]->CNT = 0U;
    }
    for (index = 0U; index < 4U; ++index)
    {
        pwm[index] = (int)pid_calc(&g_robot_wheel_pid[index], wheel_rpm[index], targets[index]);
    }
    robot_motion_write(pwm);
}
/* USER CODE END Application */

