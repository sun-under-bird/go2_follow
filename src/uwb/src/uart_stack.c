#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

#include "uart_stack.h"

#define UART_STACK_VERSION V02

#define UWB_LOG_INTERVAL_NS 1000000000L

// 限制底层串口日志频率，避免高频输出阻塞数据解析线程。
static int uwb_log_printf(const char *format, ...)
{
	static struct timespec last_log_time = {0, 0};
	struct timespec now;
	long elapsed_ns;
	va_list args;
	int result;

	clock_gettime(CLOCK_MONOTONIC, &now);
	elapsed_ns = (now.tv_sec - last_log_time.tv_sec) * 1000000000L
		+ now.tv_nsec - last_log_time.tv_nsec;
	if (last_log_time.tv_sec != 0 && elapsed_ns < UWB_LOG_INTERVAL_NS)
	{
		return 0;
	}

	last_log_time = now;
	va_start(args, format);
	result = vprintf(format, args);
	va_end(args);
	return result;
}
#define UWB_LOG uwb_log_printf
// #define UWB_LOG(...)

// 接收状态独立保存，串口重连和半帧超时均可显式复位。
typedef enum {waitForFirstStart, waitForSecondStart, waitForSeq, waitForLen,
  waitForData, waitForCrc} dataRxState;
static dataRxState rx_state = waitForFirstStart;
static uint8_t radar_rx_Buf[100];
static uint8_t rx_packet_ok = 0;
static uint16_t receive_tlv_len = 0;
static uint16_t receive_count = 0;
static uint8_t length_count = 0;
static uint8_t crc_count = 0;
static uint64_t last_byte_ms = 0;
static int have_byte_time = 0;

// 清除所有半帧状态，保证设备重连后不会拼接断线前的数据。
void uart_reset_receiver(void)
{
  rx_state = waitForFirstStart;
  rx_packet_ok = 0;
  receive_tlv_len = 0;
  receive_count = 0;
  length_count = 0;
  crc_count = 0;
  have_byte_time = 0;
}

// 计算协议使用的 CRC-CCITT，支持任意合法载荷长度。
static uint16_t math_crc16(uint16_t crc, const void *data, uint16_t len)
{
  const uint8_t *bytes = (const uint8_t *)data;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)bytes[i] << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// 按高低字节分离协议 CRC，供接收检查和发送打包复用。
static int8_t calc_crc(uint8_t *data, uint16_t len, uint8_t *high, uint8_t *low)
{
  const uint16_t crc = math_crc16(0, data, len);
  *high = (uint8_t)(crc >> 8);
  *low = (uint8_t)crc;
  return 0;
}

// 使用可注入的单调时间解析字节，超过 200 ms 的半帧立即丢弃并重新寻帧。
int8_t uart_receive_byte_at(uint8_t byte, uint64_t now_ms)
{
  if (have_byte_time && (now_ms < last_byte_ms || now_ms - last_byte_ms > 200U)) {
    uart_reset_receiver();
  }
  last_byte_ms = now_ms;
  have_byte_time = 1;
  rx_packet_ok = 0;
  switch (rx_state) {
    case waitForFirstStart:
      rx_state = byte == PACKET_HEAD0 ? waitForSecondStart : waitForFirstStart;
      break;
    case waitForSecondStart:
      // 连续 0x55 保留最后一个头字节，避免一个噪声字节吞掉完整帧头。
      rx_state = byte == PACKET_HEAD1 ? waitForSeq :
        (byte == PACKET_HEAD0 ? waitForSecondStart : waitForFirstStart);
      break;
    case waitForSeq:
      receive_tlv_len = 0;
      length_count = 0;
      rx_state = waitForLen;
      break;
    case waitForLen:
      receive_tlv_len |= (uint16_t)byte << (8U * length_count++);
      if (length_count == 2U) {
        if (receive_tlv_len < 2U || receive_tlv_len > sizeof(radar_rx_Buf) - 2U) {
          uart_reset_receiver();
          break;
        }
        receive_count = 0;
        crc_count = 0;
        rx_state = waitForData;
      }
      break;
    case waitForData:
      radar_rx_Buf[receive_count++] = byte;
      if (receive_count == receive_tlv_len) {
        rx_state = waitForCrc;
      }
      break;
    case waitForCrc:
      radar_rx_Buf[receive_tlv_len + crc_count++] = byte;
      if (crc_count == 2U) {
        const uint16_t received_crc = ((uint16_t)radar_rx_Buf[receive_tlv_len] << 8) |
          radar_rx_Buf[receive_tlv_len + 1U];
        rx_packet_ok = math_crc16(0, radar_rx_Buf, receive_tlv_len) == received_crc;
        rx_state = waitForFirstStart;
      }
      break;
  }
  return rx_packet_ok;
}

// 用系统单调时钟驱动字节解析，不受 ROS 或墙上时钟跳变影响。
int8_t uart_receive_byte(uint8_t byte)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return uart_receive_byte_at(byte, (uint64_t)now.tv_sec * 1000U + now.tv_nsec / 1000000U);
}

// 在本地缓冲区内打包协议帧；为帧头、TLV 和 CRC 预留完整空间。
void uart_protocol_transmit(uint8_t cmd, uint8_t* data, uint16_t len)
{
	static uint8_t seq = 0;
	uint16_t tlv_total_len = len + 2;
	uint8_t crc_high, crc_low = 0;
	uint8_t uart_data[255] = {0x55, 0xAA, 0x00, 0x04, 0x00, 0x00, 0x02, 0x41, 0x00, 0x00, 0x00};

	if(len > sizeof(uart_data) - 9U || (len > 0U && data == NULL)) return;

	seq++;
	
	uart_data[2] = seq;
	uart_data[3] = tlv_total_len&0xff;
	uart_data[4] = (tlv_total_len>>8)&0xff;
	uart_data[5] = cmd;
	uart_data[6] = len;
	memcpy(&uart_data[7], data, len);

	calc_crc(&uart_data[5], tlv_total_len, &crc_high, &crc_low);

	uart_data[7+len] = crc_high;
	uart_data[7+len+1] = crc_low;

	// TODO, uart transmit
	// phscaLinFlex_UartSendBytes(AskWritePulseWidthToFlash_array, 11);
}

// 验证完整载荷尺寸后才向上层暴露厂家结构，防止短帧复用缓冲区中的旧坐标。
int8_t uart_protocol_packet_process(void **buffer)
{
  if (buffer == NULL || !rx_packet_ok) {
    return 0;
  }
  *buffer = NULL;
  rx_packet_ok = 0;
  if (receive_tlv_len < 2U || radar_rx_Buf[1] > receive_tlv_len - 2U) {
    return 0;
  }
  if (radar_rx_Buf[0] == NOTIFY_DISTANCE_ANGLE_RSSI_FOBID &&
    receive_tlv_len >= 2U + sizeof(uwb_aoa_fob_pkg_t) &&
    radar_rx_Buf[1] >= sizeof(uwb_aoa_fob_pkg_t))
  {
    uwb_aoa_fob_pkg_t *packet = (uwb_aoa_fob_pkg_t *)&radar_rx_Buf[2];
    *buffer = packet;
    UWB_LOG("[UWB]: dis: %.3f, agl: %.3f, pitch: %.3f\n",
      packet->distance, packet->angle, packet->pitch);
    return (int8_t)NOTIFY_DISTANCE_ANGLE_RSSI_FOBID;
  }
  return 0;
}
