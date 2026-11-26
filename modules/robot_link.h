/* 作者：Feng-100M
 * 文件名：robot_link.h
 * 用途：定义有界串口协议、会话和同步业务回调。
 * 日期：2026-11-25
 */
#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_LINK_MAX_PAYLOAD (32U)
#define ROBOT_LINK_TIMEOUT_MS (500U)
#define ROBOT_LINK_ACK_SIZE (8U)

typedef struct
{
    bool heartbeat;
    bool has_spray;
    bool spray_on;
    bool duplicate;
    bool legacy;
    uint8_t sequence;
} robot_link_event;

typedef enum
{
    /* 等待行首的协议标记。 */
    ROBOT_LINK_WAIT_START = 0,
    /* 接收以换行结束的协议行。 */
    ROBOT_LINK_READ_LINE,
    /* 数据有缺口或超长，丢弃至下一换行。 */
    ROBOT_LINK_DROP_LINE
} robot_link_phase;

typedef struct
{
    char line[100];
    uint8_t used;
    robot_link_phase phase;
    bool active;
    uint32_t heartbeat_ms;
    uint32_t errors;

    uint8_t last_frame[2U + ROBOT_LINK_MAX_PAYLOAD];
    uint8_t last_size;
} robot_link_receiver;

/* 请求行以 ! 开头：序号(1 B)、载荷长度(1 B)、TLV 载荷(至多 32 B)、CRC(2 B)。
 * 各字节以大写十六进制传输，CRC 高字节在前，行尾接受 LF 或 CRLF。
 * CRC-16/CCITT-FALSE 覆盖二进制序号、长度和载荷；ACK 为 +、序号、仅序号的 CRC 和 LF。
 * TLV 的类型和长度各 1 B：01 00 为心跳，02 01 00/01 为关泵/开泵。
 * 未知类型按长度跳过；需至少一个已知字段，已知类型重复或长度错误则整帧丢弃。
 * 首帧须含心跳且无泵指令，起始序号任选。后续序号按模 256 递增。
 * 上一帧原样重发只确认，新序号的心跳才续期。发送端单独重启不会重建会话。
 *
 * 单任务独占零初始化的接收器。now_ms 仅在处理换行时使用，取该帧解析时刻。
 * 完整有效帧同步调用回调，事件指针只在回调内有效。错误帧计入 errors。
 * 空指针不处理。重复帧仅确认，调用方必须检查 duplicate。
 */
void robot_link_feed(robot_link_receiver *receiver, uint8_t byte, uint32_t now_ms,
                     void (*deliver)(const robot_link_event *));

/* 已建立会话满 500 毫秒未收到新心跳时为真。时钟允许 32 位回绕。
 * 由接收任务调用。空指针视为超时。
 */
bool robot_link_expired(const robot_link_receiver *receiver, uint32_t now_ms);

/* 丢弃会话和半帧，保留错误计数。由唯一接收任务调用。 */
void robot_link_reset(robot_link_receiver *receiver);

/* 将指定序号编码为 8 字节确认。调用者独占至少 8 字节的输出缓冲。
 * 无阻塞，可重入。空指针时返回 false。
 */
bool robot_link_ack(uint8_t sequence, uint8_t output[ROBOT_LINK_ACK_SIZE]);

#endif /* ROBOT_LINK_H 结束 */
