/* 作者：Feng-100M
 * 文件名：robot_link.c
 * 用途：逐字节组帧、CRC 校验、TLV 分发及停等协议去重。
 * 日期：2026-11-25
 */
#include <stddef.h>
#include <string.h>

#include "./robot_link.h"

#define ROBOT_LINK_CRC_POLYNOMIAL (0x1021U)
#define ROBOT_LINK_CRC_INITIAL (0xffffU)
#define ROBOT_LINK_HEARTBEAT (1U)
#define ROBOT_LINK_SPRAY (2U)

typedef struct
{
    uint8_t type;
    uint8_t length;
    bool (*decode)(const uint8_t *, robot_link_event *);
} robot_link_rule;

static bool robot_link_heartbeat(const uint8_t *value, robot_link_event *event)
{
    (void)value;
    if (event->heartbeat)
    {
        return false;
    }
    event->heartbeat = true;
    return true;
}

static bool robot_link_spray(const uint8_t *value, robot_link_event *event)
{
    if (event->has_spray || value[0] > 1U)
    {
        return false;
    }
    event->has_spray = true;
    event->spray_on = value[0] != 0U;
    return true;
}

/* 协议版本内的只读命令表，与固件同生命周期。 */
static const robot_link_rule g_robot_link_rules[] = {
    {ROBOT_LINK_HEARTBEAT, 0U, robot_link_heartbeat},
    {ROBOT_LINK_SPRAY, 1U, robot_link_spray}
};

/* 计算 bytes 中 size 个字节的 CRC-16，初值 0xFFFF，多项式 0x1021，高位先行。 */
static uint16_t robot_link_crc(const uint8_t *bytes, size_t size)
{
    uint16_t crc = ROBOT_LINK_CRC_INITIAL;
    size_t index;
    unsigned int bit;

    for (index = 0U; index < size; ++index)
    {
        crc ^= (uint16_t)((uint16_t)bytes[index] << 8);
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (uint16_t)((crc & 0x8000U) != 0U ?
                             ((uint32_t)crc << 1) ^ ROBOT_LINK_CRC_POLYNOMIAL :
                             ((uint32_t)crc << 1));
        }
    }
    return crc;
}

bool robot_link_ack(uint8_t sequence, uint8_t output[ROBOT_LINK_ACK_SIZE])
{
    const char digits[] = "0123456789ABCDEF";
    uint16_t crc = robot_link_crc(&sequence, 1U);
    if (output == NULL)
    {
        return false;
    }
    output[0] = '+';
    output[1] = (uint8_t)digits[sequence >> 4];
    output[2] = (uint8_t)digits[sequence & 15U];
    output[3] = (uint8_t)digits[crc >> 12];
    output[4] = (uint8_t)digits[(crc >> 8) & 15U];
    output[5] = (uint8_t)digits[(crc >> 4) & 15U];
    output[6] = (uint8_t)digits[crc & 15U];
    output[7] = '\n';
    return true;
}

static int robot_link_digit(char byte)
{
    if (byte >= '0' && byte <= '9')
    {
        return byte - '0';
    }
    if (byte >= 'A' && byte <= 'F')
    {
        return byte - 'A' + 10;
    }
    return -1;
}

/* 所有字段先写入栈上事件，尾部任何错误都会撤销整帧。 */
static bool robot_link_decode(const uint8_t *bytes, size_t end, robot_link_event *event)
{
    size_t offset = 2U;
    while (offset < end)
    {
        size_t index;
        uint8_t type;
        uint8_t size;
        if (end - offset < 2U)
        {
            return false;
        }
        type = bytes[offset++];
        size = bytes[offset++];
        if (size > end - offset)
        {
            return false;
        }
        for (index = 0U; index < sizeof(g_robot_link_rules) / sizeof(g_robot_link_rules[0]); ++index)
        {
            const robot_link_rule *rule = &g_robot_link_rules[index];
            if (type == rule->type &&
                (size != rule->length || !rule->decode(&bytes[offset], event)))
            {
                return false;
            }
        }
        offset += size;
    }
    return event->has_spray || event->heartbeat;
}

/* 校验一行十六进制数据的长度、CRC、命令和会话序号；调用方提供清零的临时 event。
 * 会话须由含心跳且无施药命令的帧建立。同序号只有内容也相同时才作为重复帧交付。
 * 返回 false 时调用方丢弃整个 event，接收器保留上次接受的帧。
 */
static bool robot_link_frame(robot_link_receiver *receiver, robot_link_event *event)
{
    uint8_t bytes[ROBOT_LINK_MAX_PAYLOAD + 4U];
    size_t count = (receiver->used - 1U) / 2U;
    size_t index;
    uint16_t crc;
    if ((receiver->used & 1U) == 0U || count < 4U || count > sizeof(bytes))
    {
        return false;
    }
    for (index = 0U; index < count; ++index)
    {
        int high = robot_link_digit(receiver->line[1U + 2U * index]);
        int low = robot_link_digit(receiver->line[2U + 2U * index]);
        if (high < 0 || low < 0)
        {
            return false;
        }
        bytes[index] = (uint8_t)((unsigned int)high * 16U + (unsigned int)low);
    }
    count -= 2U;
    crc = robot_link_crc(bytes, count);
    if (bytes[1] != count - 2U || bytes[count] != (uint8_t)(crc >> 8) ||
        bytes[count + 1U] != (uint8_t)crc || !robot_link_decode(bytes, count, event))
    {
        return false;
    }
    event->sequence = bytes[0];
    if (receiver->active)
    {
        if (event->sequence == receiver->last_frame[0])
        {
            event->duplicate = count == receiver->last_size &&
                               memcmp(bytes, receiver->last_frame, count) == 0;
            return event->duplicate;
        }
        if (event->sequence != (uint8_t)(receiver->last_frame[0] + 1U))
        {
            return false;
        }
    }
    else if (!event->heartbeat || event->has_spray)
    {
        return false;
    }
    memcpy(receiver->last_frame, bytes, count);
    receiver->last_size = (uint8_t)count;
    receiver->active = true;

    return true;
}

bool robot_link_expired(const robot_link_receiver *receiver, uint32_t now_ms)
{
    return receiver == NULL || (receiver->active &&
           (uint32_t)(now_ms - receiver->heartbeat_ms) >= ROBOT_LINK_TIMEOUT_MS);
}

void robot_link_reset(robot_link_receiver *receiver)
{
    uint32_t errors;
    if (receiver == NULL)
    {
        return;
    }
    errors = receiver->errors;
    memset(receiver, 0, sizeof(*receiver));
    receiver->errors = errors;
}

void robot_link_feed(robot_link_receiver *receiver, uint8_t byte, uint32_t now_ms,
                     void (*deliver)(const robot_link_event *))
{
    robot_link_event event = {0};
    size_t index;
    if (receiver == NULL || deliver == NULL)
    {
        return;
    }
    if (byte != '\n')
    {
        if (receiver->phase == ROBOT_LINK_WAIT_START)
        {
            receiver->phase = (byte == '!' || byte == '@') ? ROBOT_LINK_READ_LINE : ROBOT_LINK_DROP_LINE;
        }
        if (receiver->used == sizeof(receiver->line))
        {
            receiver->phase = ROBOT_LINK_DROP_LINE;
        }
        if (receiver->phase == ROBOT_LINK_READ_LINE)
        {
            receiver->line[receiver->used++] = (char)byte;
        }
        return;
    }
    if (receiver->used != 0U && receiver->line[receiver->used - 1U] == '\r')
    {
        --receiver->used;
    }
    if (receiver->phase == ROBOT_LINK_DROP_LINE || robot_link_expired(receiver, now_ms))
    {
        ++receiver->errors;
    }
    else if (receiver->used != 0U && receiver->line[0] == '!')
    {
        if (robot_link_frame(receiver, &event))
        {
            if (event.heartbeat && !event.duplicate)
            {
                receiver->heartbeat_ms = now_ms;
            }
            deliver(&event);
        }
        else
        {
            ++receiver->errors;
        }
    }
    else if (!receiver->active && receiver->used != 0U && receiver->line[0] == '@')
    {
        for (index = 1U; index < receiver->used; ++index)
        {
            if (receiver->line[index] == 'd' || receiver->line[index] == 'n')
            {
                event.has_spray = true;
                event.spray_on = receiver->line[index] == 'd';
            }
        }
        if (event.has_spray)
        {
            event.legacy = true;
            deliver(&event);
        }
    }
    receiver->used = 0U;
    receiver->phase = ROBOT_LINK_WAIT_START;
}
