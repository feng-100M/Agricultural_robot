/* 作者：Feng-100M
 * 文件名：robot_uart.h
 * 用途：管理 USART2 循环 DMA 和接收任务通知。
 * 日期：2026-11-25
 */
#ifndef ROBOT_UART_H
#define ROBOT_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 从唯一通信任务启动或恢复接收，清空旧字节。成功为 true。
 * 绑定当前任务用于中断通知。失败时不得消费旧数据。
 */
bool robot_uart_start(void);

/* 短临界区复制最多 capacity 字节；返回复制长度。fault 必须有效，
 * 为真表示数据流有缺口，需要重启接收并丢弃至下一换行。
 * 单消费者任务调用，输出缓冲由调用者持有。
 */
size_t robot_uart_read(uint8_t *output, size_t capacity, bool *fault);

#endif /* ROBOT_UART_H 结束 */
