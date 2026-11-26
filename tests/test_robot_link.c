/* 作者：Feng-100M
 * 文件名：test_robot_link.c
 * 用途：验证通信帧边界、重复确认、原子分发和通信超时。
 * 日期：2026-11-25
 */
#ifdef NDEBUG
#error "Tests require enabled assertions"
#endif /* NDEBUG 条件结束 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../modules/robot_link.h"

/* 同步回调与主测试共用一组状态，计数从进程启动累计至退出。 */
static unsigned int g_test_robot_link_actions;
static unsigned int g_test_robot_link_acks;
static unsigned int g_test_robot_link_legacy;
static robot_link_receiver g_test_robot_link_receiver;
static robot_link_event g_test_robot_link_event;

/* event 仅在回调期间有效，复制后供断言检查。
 * 每次新协议交付均计入待确认次数，只有非重复帧计入业务动作次数。
 */
static void test_record(const robot_link_event *event)
{
    g_test_robot_link_event = *event;
    if (event->legacy)
    {
        ++g_test_robot_link_legacy;
    }
    else
    {
        ++g_test_robot_link_acks;
        if (!event->duplicate)
        {
            ++g_test_robot_link_actions;
        }
    }
}

/* 将以空字符结尾的测试片段逐字节交给同一接收器。
 * 同一片段共用 now_ms 毫秒时间戳，连续调用可模拟串口分批接收。
 */
static void test_feed(const char *bytes, uint32_t now_ms)
{
    while (*bytes != '\0')
    {
        robot_link_feed(&g_test_robot_link_receiver, (uint8_t)*bytes, now_ms, test_record);
        ++bytes;
    }
}

/* 使用预先计算的帧和 ACK 验证协议。全部断言通过返回 0。 */
int main(void)
{
    uint8_t ack[ROBOT_LINK_ACK_SIZE];
    unsigned int index;
    assert(robot_link_ack(0xfeU, ack));
    assert(memcmp(ack, "+FEEF21\n", ROBOT_LINK_ACK_SIZE) == 0);
    /* 独立 CRC 参考向量。先拆开心跳，再紧接一帧含未知字段的命令。 */
    test_feed("!FE0201", 20U);
    assert(g_test_robot_link_actions == 0U);
    test_feed("00E486\n!FF06770113020101EED3\n", 21U);
    assert(g_test_robot_link_actions == 2U && g_test_robot_link_acks == 2U);
    assert(g_test_robot_link_event.has_spray && g_test_robot_link_event.spray_on && !g_test_robot_link_event.duplicate);
    /* 模拟 ACK 丢失后重传同一帧：接收器再次交付以便确认，业务动作仍只执行一次。 */
    test_feed("!FF06770113020101EED3\n", 30U);
    assert(g_test_robot_link_actions == 2U && g_test_robot_link_acks == 3U);
    /* 同序号不同内容、错误 CRC、越界 TLV 和超长声明一律丢弃。 */
    test_feed("!FF030201008D2E\n!00030201000000\n!000302020082D2\n!10FF00\n", 31U);
    assert(g_test_robot_link_actions == 2U && g_test_robot_link_acks == 3U);
    test_feed("!0003020100D781\n", 32U);
    assert(g_test_robot_link_actions == 3U);
    test_feed("@cmd\r\n@n\n", 35U);
    assert(g_test_robot_link_legacy == 0U);
    /* 最后一次新心跳在 21 ms，随后收到的施药命令不续期。 */
    assert(!robot_link_expired(&g_test_robot_link_receiver, 520U));
    assert(robot_link_expired(&g_test_robot_link_receiver, 521U));
    assert(g_test_robot_link_receiver.errors >= 4U);
    test_feed("!01030201016DF1\n", 522U);
    assert(g_test_robot_link_actions == 3U && g_test_robot_link_acks == 4U);
    robot_link_reset(&g_test_robot_link_receiver);
    test_feed("@cmd\r\n@n\n", UINT32_MAX - 10U);
    assert(g_test_robot_link_legacy == 2U);
    assert(!robot_link_expired(&g_test_robot_link_receiver, 600U));
    /* 32 位毫秒时钟回绕后，488 和 489 分别距该心跳 499 和 500 毫秒。 */
    test_feed("!FE020100E486\n", UINT32_MAX - 10U);
    assert(!robot_link_expired(&g_test_robot_link_receiver, 488U));
    assert(robot_link_expired(&g_test_robot_link_receiver, 489U));
    robot_link_reset(&g_test_robot_link_receiver);
    /* 超长旧协议行必须整行丢弃，下一行的有效心跳应能重新建立会话。 */
    test_feed("@", 1U);
    for (index = 0U; index < 400U; ++index)
    {
        test_feed("d", 1U);
    }
    test_feed("\n!00020100D991\n!010801007701130201001AF6\n", 2U);
    assert(g_test_robot_link_legacy == 2U && g_test_robot_link_actions == 6U);
    assert(g_test_robot_link_event.heartbeat && g_test_robot_link_event.has_spray && !g_test_robot_link_event.spray_on);
    /* 当前会话已确认 01。发送端重启回到 00 不得重建会话，重复 01 可确认但不续期。 */
    {
        unsigned int actions = g_test_robot_link_actions;
        unsigned int acks = g_test_robot_link_acks;
        test_feed("!00020100D991\n", 490U);
        assert(g_test_robot_link_acks == acks);
        test_feed("!010801007701130201001AF6\n", 501U);
        assert(g_test_robot_link_event.duplicate && g_test_robot_link_acks == acks + 1U);
        assert(!robot_link_expired(&g_test_robot_link_receiver, 501U));
        assert(robot_link_expired(&g_test_robot_link_receiver, 502U));
        test_feed("!0202010034F9\n", 502U);
        assert(g_test_robot_link_acks == acks + 1U && g_test_robot_link_actions == actions);
        robot_link_reset(&g_test_robot_link_receiver);
        test_feed("!0202010034F9\n", 503U);
        assert(g_test_robot_link_acks == acks + 2U && g_test_robot_link_actions == actions + 1U);
    }
    puts("robot_link: PASS");
    return 0;
}
