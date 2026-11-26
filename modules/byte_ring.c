/* 作者：Feng-100M
 * 文件名：byte_ring.c
 * 用途：实现 USART 中断生产者与通信任务消费者共用的字节队列。
 * 日期：2026-11-25
 */
#include <stddef.h>

#include "./byte_ring.h"

void byte_ring_reset(byte_ring *ring)
{
    if (ring != NULL)
    {
        ring->head = 0U;
        ring->tail = 0U;
        ring->dropped = 0U;
    }
}

bool byte_ring_push(byte_ring *ring, uint8_t byte)
{
    uint16_t head;
    uint16_t next_head;
    if (ring == NULL)
    {
        return false;
    }
    head = ring->head;
    next_head = (uint16_t)((head + 1U) % BYTE_RING_CAPACITY);
    if (next_head == ring->tail)
    {
        ++ring->dropped;
        return false;
    }
    ring->bytes[head] = byte;
    ring->head = next_head;
    return true;
}

bool byte_ring_pop(byte_ring *ring, uint8_t *byte)
{
    uint16_t tail;
    if (ring == NULL || byte == NULL)
    {
        return false;
    }
    tail = ring->tail;
    if (tail == ring->head)
    {
        return false;
    }
    *byte = ring->bytes[tail];
    ring->tail = (uint16_t)((tail + 1U) % BYTE_RING_CAPACITY);
    return true;
}
