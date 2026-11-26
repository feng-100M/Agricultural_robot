/* 作者：Feng-100M
 * 文件名：test_byte_ring.c
 * 用途：验证字节环形缓冲区空、满、回绕和溢出行为。
 * 日期：2026-11-25
 */
#ifdef NDEBUG
#error "Tests require enabled assertions"
#endif /* NDEBUG 条件结束 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../modules/byte_ring.h"

/* 在单线程中交替写入和取出，验证回绕后两批字节的先后顺序。
 * 所有断言通过时返回 0，断言失败时由主机运行库终止测试。
 */
int main(void)
{
    byte_ring ring;
    uint8_t byte = 0U;
    unsigned int index;
    byte_ring_reset(&ring);
    assert(!byte_ring_pop(&ring, &byte));
    /* 环形队列预留一个槽以区分空和满，填入 BYTE_RING_CAPACITY - 1 个字节后应拒绝新字节。 */
    for (index = 0U; index < BYTE_RING_CAPACITY - 1U; ++index)
    {
        assert(byte_ring_push(&ring, (uint8_t)index));
    }
    assert(!byte_ring_push(&ring, 0U));
    assert(ring.dropped == 1U);

    /* 先腾出半圈再写入第二批，使写指针越过数组末端，同时保留第一批尚未读取的字节。 */
    for (index = 0U; index < BYTE_RING_CAPACITY / 2U; ++index)
    {
        assert(byte_ring_pop(&ring, &byte));
        assert(byte == (uint8_t)index);
    }
    for (index = 0U; index < BYTE_RING_CAPACITY / 2U; ++index)
    {
        assert(byte_ring_push(&ring, (uint8_t)(index + 7U)));
    }
    for (index = BYTE_RING_CAPACITY / 2U; index < BYTE_RING_CAPACITY - 1U; ++index)
    {
        assert(byte_ring_pop(&ring, &byte));
        assert(byte == (uint8_t)index);
    }
    for (index = 0U; index < BYTE_RING_CAPACITY / 2U; ++index)
    {
        assert(byte_ring_pop(&ring, &byte));
        assert(byte == (uint8_t)(index + 7U));
    }
    assert(!byte_ring_pop(&ring, &byte));
    puts("byte_ring: PASS");
    return 0;
}
