/* 作者：Feng-100M
 * 文件名：byte_ring.h
 * 用途：定义固定容量、单生产者单消费者字节环形缓冲区。
 * 日期：2026-11-25
 */
#ifndef BYTE_RING_H
#define BYTE_RING_H

#include <stdbool.h>
#include <stdint.h>

#define BYTE_RING_CAPACITY (512U)

typedef struct
{
    volatile uint8_t bytes[BYTE_RING_CAPACITY];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t dropped;
} byte_ring;

/* 仅在生产者、消费者均停止时调用，清空缓冲区和溢出计数。 */
void byte_ring_reset(byte_ring *ring);

/* ISR 单生产者写入一个字节。满时丢弃新字节、递增 dropped 并返回 false。 */
bool byte_ring_push(byte_ring *ring, uint8_t byte);

/* 唯一消费者必须屏蔽生产者中断后读取。空时返回 false，保留输出。 */
bool byte_ring_pop(byte_ring *ring, uint8_t *byte);

#endif /* BYTE_RING_H 结束 */
