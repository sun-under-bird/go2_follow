#ifndef _UART_STACK_H
#define _UART_STACK_H

#include <stdint.h>

#define PACKET_HEAD0	(0X55)
#define PACKET_HEAD1	(0xAA)

//Read And Write
#define DEVICE_RESPONSE	(0x00)
#define READ_PRODUCT_ID	(0x02)
#define CFG_KEY			(0x03)
#define CFG_DEVICE_MODE	(0x04)
#define CFG_DETECT_DIS	(0x40)
#define CFG_PW_WIDTH	(0x41)
#define CFG_SYS_RESET	(0x05)

//Notify
#define NOTIFY_DISTANCE_ANGLE_RSSI      	(0xC4)
#define NOTIFY_DISTANCE_ANGLE_RSSI_FOBID	(0xC5)

// data process
#define SET_BIT_FIELD(data, mask, pos, value) (((data) & ~mask) | ((uint32_t)((value) << pos) & mask))
#define GET_BIT_FIELD(data, mask, pos) (((data)&mask) >> pos)

// register aceess
#define REG_READ_BYTE(addr) (*(const volatile uint8_t *)(addr))
#define REG_WRITE_BYTE(addr, value) (*(volatile uint8_t *)(addr) = (value))

#define REG_READ(addr) (*(const volatile uint32_t *)(addr))
#define REG_WRITE(addr, value) (*(volatile uint32_t *)(addr) = (value))

#define REG_MODIFY(addr, mask, value) REG_WRITE(addr, (REG_READ(addr) & (uint32_t)~mask) | value)
#define REG_READ_BIT_FIELD(addr, width, pos) ((REG_READ(addr) >> pos) & ((1 << (width)) - 1))

// read word from an unaligned address
#define READ_WORD(x) ((uint32_t)(REG_READ_BYTE(x) | (REG_READ_BYTE(x + 1) << 8) | (REG_READ_BYTE(x + 2) << 16) | (REG_READ_BYTE(x + 3) << 24)))
// read half word from an unaligned address
#define READ_SHORT(x) ((uint16_t)(REG_READ_BYTE(x) | (REG_READ_BYTE(x + 1) << 8)))


//device response, type 00
typedef struct 
{
	uint8_t type_resp;
	uint8_t status;

}uwb_device_response_t;

// type C4
typedef struct
{
	float distance;
	float angle;
	float pitch;
	uint8_t rssi_len;
	int8_t rssi[6];

//}__attribute__((packed)) uwb_aoa_pkg_t;
} uwb_aoa_pkg_t;//TODO, no align

// type C5
typedef struct
{
	uint32_t sync_cnt;
	uint32_t index;
	uint32_t fob_id;
	uint16_t fob_type;
	float distance;
	float angle;
	float pitch;
	uint8_t rssi_len;
	int8_t rssi[6];
	int8_t rx_power;
	int8_t rssi_fpp;
	int8_t rssi_np;
	int8_t rssi_ble;
	uint8_t pos_confidence;

}__attribute__((packed)) uwb_aoa_fob_pkg_t;
// } uwb_aoa_fob_pkg_t;//TODO, no align

void uart_protocol_transmit(uint8_t cmd, uint8_t* data, uint16_t len);
int8_t uart_receive_byte(uint8_t input_data);
int8_t uart_protocol_packet_process(void **buffer);

#endif
