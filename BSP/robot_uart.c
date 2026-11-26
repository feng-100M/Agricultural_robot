/* 作者：Feng-100M
 * 文件名：robot_uart.c
 * 用途：手动配置 STM32H750 USART2 RX 的 DMAMUX 和 DMA1 Stream0。
 * 日期：2026-11-25
 */
#include <stddef.h>

#include "./robot_uart.h"
#include "../Core/Inc/usart.h"
#include "../modules/byte_ring.h"
#include "FreeRTOS.h"
#include "task.h"

/* RM0433 的 D2 SRAM1 起始地址。当前 MDK 链接区只使用 DTCM 和 AXI SRAM。
 * 独占前 256 字节，按 Cortex-M7 的 32 字节缓存行对齐。
 */
#define ROBOT_UART_DMA_BASE (0x30000000UL)
#define ROBOT_UART_DMA_SIZE (256U)
#define ROBOT_UART_READ_LIMIT (32U)

/* DMA 及队列对象常驻。两个生产者中断同优先级，消费者屏蔽两者后出队。 */
static DMA_HandleTypeDef g_robot_uart_dma;
static byte_ring g_robot_uart_ring;
static TaskHandle_t g_robot_uart_task;
static uint16_t g_robot_uart_position;
static volatile bool g_robot_uart_fault;

static void robot_uart_notify(void)
{
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(g_robot_uart_task, &wake);
    portYIELD_FROM_ISR(wake);
}

void DMA1_Stream0_IRQHandler(void);
/* 处理 USART2 接收 DMA 中断。半满和全满标志同时积压时锁存字节流故障，
 * 通知通信任务后仍交由 HAL 处理本次中断。
 */
void DMA1_Stream0_IRQHandler(void)
{
    /* 两个半区中断均积压时，无法判断 DMA 是否已经多次回绕。 */
    if (__HAL_DMA_GET_FLAG(&g_robot_uart_dma, __HAL_DMA_GET_HT_FLAG_INDEX(&g_robot_uart_dma)) &&
        __HAL_DMA_GET_FLAG(&g_robot_uart_dma, __HAL_DMA_GET_TC_FLAG_INDEX(&g_robot_uart_dma)))
    {
        g_robot_uart_fault = true;
        robot_uart_notify();
    }
    HAL_DMA_IRQHandler(&g_robot_uart_dma);
}

/* 在 USART2 或 DMA 中断中将新到达的字节复制到环形队列，再唤醒通信任务。
 * 空闲、半满和全满事件可能紧邻，因此使用实时 NDTR 确定写入位置。队列满时锁存故障。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    const volatile uint8_t *memory = (const volatile uint8_t *)ROBOT_UART_DMA_BASE;
    uint16_t position;
    (void)size;
    if (uart != &huart2 || g_robot_uart_fault || g_robot_uart_task == NULL)
    {
        return;
    }
    position = (uint16_t)((ROBOT_UART_DMA_SIZE - __HAL_DMA_GET_COUNTER(&g_robot_uart_dma)) %
                          ROBOT_UART_DMA_SIZE);
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)ROBOT_UART_DMA_BASE, ROBOT_UART_DMA_SIZE);
    }
    while (g_robot_uart_position != position)
    {
        if (!byte_ring_push(&g_robot_uart_ring, memory[g_robot_uart_position]))
        {
            g_robot_uart_fault = true;
            break;
        }
        g_robot_uart_position = (uint16_t)((g_robot_uart_position + 1U) % ROBOT_UART_DMA_SIZE);
    }
    robot_uart_notify();
}

/* 在 HAL 中断回调中锁存 USART2 接收错误并通知已绑定的通信任务，由任务重启接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart2 && g_robot_uart_task != NULL)
    {
        g_robot_uart_fault = true;
        robot_uart_notify();
    }
}

bool robot_uart_start(void)
{
    HAL_StatusTypeDef status;
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    g_robot_uart_fault = true;
    if (g_robot_uart_task != NULL)
    {
        if (HAL_UART_AbortReceive(&huart2) != HAL_OK)
        {
            return false;
        }
        if (HAL_DMA_DeInit(&g_robot_uart_dma) != HAL_OK)
        {
            return false;
        }
    }
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    g_robot_uart_dma.Instance = DMA1_Stream0;
    g_robot_uart_dma.Init.Request = DMA_REQUEST_USART2_RX;
    g_robot_uart_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_robot_uart_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    g_robot_uart_dma.Init.MemInc = DMA_MINC_ENABLE;
    g_robot_uart_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_robot_uart_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_robot_uart_dma.Init.Mode = DMA_CIRCULAR;
    g_robot_uart_dma.Init.Priority = DMA_PRIORITY_HIGH;
    g_robot_uart_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&g_robot_uart_dma) != HAL_OK)
    {
        return false;
    }
    __HAL_LINKDMA(&huart2, hdmarx, g_robot_uart_dma);
    byte_ring_reset(&g_robot_uart_ring);
    g_robot_uart_position = 0U;
    g_robot_uart_task = xTaskGetCurrentTaskHandle();
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)ROBOT_UART_DMA_BASE, ROBOT_UART_DMA_SIZE);
    }
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)ROBOT_UART_DMA_BASE, ROBOT_UART_DMA_SIZE);
    if (status != HAL_OK)
    {
        return false;
    }
    g_robot_uart_fault = false;
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return true;
}

size_t robot_uart_read(uint8_t *output, size_t capacity, bool *fault)
{
    size_t count = 0U;
    if (fault == NULL)
    {
        return 0U;
    }
    taskENTER_CRITICAL();
    *fault = g_robot_uart_fault || output == NULL;
    while (!*fault && count < capacity && count < ROBOT_UART_READ_LIMIT &&
           byte_ring_pop(&g_robot_uart_ring, &output[count]))
    {
        ++count;
    }
    taskEXIT_CRITICAL();
    return count;
}
