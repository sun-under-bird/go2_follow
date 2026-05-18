#include <string.h>
#include <stdio.h>

#include "uart_stack.h"

#define UART_STACK_VERSION V02

#define UWB_LOG printf
// #define UWB_LOG(...)

//uart receive
typedef enum
{
	waitForFirstStart = 0x00,
	waitForSecondStart,
	waitForSeq,
	waitForLen,
	waitForData,
	waitForCrc,
	waitForOver

} dataRxState;

static uint8_t receive_tlv_seq = 0;
static uint8_t radar_rx_Buf[100] = {0x00};
static uint8_t rx_packet_ok = 0x00;
static uint16_t receive_tlv_len = 0;
static uint16_t new_crc_data = 0x0000;

//system counter
static uint32_t sys_cnt = 0;
//extern uint32_t sys_cnt;


static uint16_t math_crc16(uint16_t last_crc_result, const void *data, uint16_t len)
{
	const static uint16_t crc_tab[16] =
		{
			0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
			0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF};

	uint8_t temporary_variable = 0;
	uint16_t crc_result = 0;
	const uint8_t *ptr = (const uint8_t *)data;

	while (len--)
	{
		temporary_variable = (uint8_t)(last_crc_result >> 12);
		last_crc_result <<= 4;
		last_crc_result ^= crc_tab[temporary_variable ^ ((*ptr) >> 4)];
		temporary_variable = last_crc_result >> 12;
		last_crc_result <<= 4;
		last_crc_result ^= crc_tab[temporary_variable ^ ((*ptr) & 0x0F)];

		crc_result = last_crc_result;
		ptr++;
	}
	return crc_result;
}

// Calc the CRC for the parameters
static int8_t calc_crc(uint8_t *data, uint16_t datat_len, uint8_t *crc_high, uint8_t *crc_low)
{
	//static uint8_t tail_crc_data[2] = {0x00, 0x00};

	new_crc_data = math_crc16(0x0000, data, datat_len);

	// tail_crc_data[0] = (uint8_t)((new_crc_data) & 0x00FF);
	// tail_crc_data[1] = (uint8_t)((new_crc_data) >> 8);

	*crc_low = (uint8_t)((new_crc_data) & 0x00FF);
	*crc_high = (uint8_t)((new_crc_data) >> 8);

	return 0;
}

// when uart interrupt receive data, call the function
int8_t uart_receive_byte(uint8_t input_data)
{
	static dataRxState rxState = waitForFirstStart;
	static uint8_t rec_tlvlen_count, rec_crc_count, rec_count = 0;
	static uint8_t rec_tlvlen_buf[2] = {0x00, 0x00};
	static uint16_t cal_rec_crc_data, rec_crc_result = 0x0000;

	static uint8_t cal_rec_crc_data_high, cal_rec_crc_data_low = 0x00;

	switch (rxState)
	{
	case waitForFirstStart:
	{
		rxState = (input_data == PACKET_HEAD0) ? waitForSecondStart : waitForFirstStart;
		break;
	}

	case waitForSecondStart:
	{
		rxState = (input_data == PACKET_HEAD1) ? waitForSeq : waitForFirstStart;
		break;
	}

	case waitForSeq:
	{
		receive_tlv_seq = input_data;
		rxState = waitForLen;
		break;
	}

	case waitForLen:
	{
		rec_tlvlen_count++;
		if (rec_tlvlen_count >= 2)
		{
			rec_tlvlen_buf[1] = input_data;

			receive_tlv_len = (uint16_t)(rec_tlvlen_buf[1] << 8) + (uint16_t)(rec_tlvlen_buf[0] << 0);

			rec_tlvlen_count = 0;
			rxState = waitForData;
		}
		rec_tlvlen_buf[0] = input_data;
		break;
	}

	case waitForData:
	{
		if (rec_count < receive_tlv_len)
		{
			radar_rx_Buf[rec_count] = input_data;
			rec_count++;
			if (rec_count >= receive_tlv_len)
			{
				rxState = waitForCrc;
			}
		}
		break;
	}

	case waitForCrc:
	{
		radar_rx_Buf[receive_tlv_len + rec_crc_count] = input_data;

		rec_crc_count++;

		if (rec_crc_count >= 2)
		{
			rec_crc_result = (uint16_t)(radar_rx_Buf[receive_tlv_len] << 8) +
							 (uint16_t)(radar_rx_Buf[receive_tlv_len + 1]);

			calc_crc(&radar_rx_Buf[0], receive_tlv_len, &cal_rec_crc_data_high, &cal_rec_crc_data_low);

			cal_rec_crc_data = (uint16_t)(cal_rec_crc_data_high << 8) + (uint16_t)(cal_rec_crc_data_low);

			if (cal_rec_crc_data == rec_crc_result)
			{
				rx_packet_ok = 0x01;
			}
			rec_count = 0;
			rec_crc_count = 0;
			rxState = waitForFirstStart;
		}
		break;
	}

	case waitForOver:
	{
		// rxState = waitForFirstStart;
		break;
	}
	}

	return rx_packet_ok;
	// return 1;
}

void uart_protocol_transmit(uint8_t cmd, uint8_t* data, uint16_t len)
{
	static uint8_t seq = 0;
	uint16_t tlv_total_len = len + 2;
	uint8_t crc_high, crc_low = 0;
	uint8_t uart_data[255] = {0x55, 0xAA, 0x00, 0x04, 0x00, 0x00, 0x02, 0x41, 0x00, 0x00, 0x00};

	if(len>255) return;

	seq++;
	
	uart_data[2] = seq;
	uart_data[3] = tlv_total_len&0xff;
	uart_data[4] = (tlv_total_len>>7)&0xff;
	uart_data[5] = cmd;
	uart_data[6] = len;
	memcpy(&uart_data[7], data, len);

	calc_crc(&uart_data[5], tlv_total_len, &crc_high, &crc_low);

	uart_data[7+len] = crc_high;
	uart_data[7+len+1] = crc_low;

	// TODO, uart transmit
	// phscaLinFlex_UartSendBytes(AskWritePulseWidthToFlash_array, 11);
}

// when uart receive a complete packet, call the function
int8_t uart_protocol_packet_process(void **buffer)
{
	uint8_t cmd = radar_rx_Buf[0];	//type
	uint8_t len = radar_rx_Buf[1];	//len
	uint8_t id = 0;					//CAN/CANFD ID	

	uint8_t ret = 1;

	if (rx_packet_ok != 0x01)
	{
		return 0;
		
	}else if (rx_packet_ok == 0x01)
	{
		switch (cmd)
		{
			case DEVICE_RESPONSE:
			{
				break;
			}
			case NOTIFY_DISTANCE_ANGLE_RSSI:
			{
				uwb_aoa_pkg_t *uwb_aoa_pkg = (uwb_aoa_pkg_t *)&radar_rx_Buf[2];
				
				UWB_LOG("[UWB UART]: dis: %.3f, agl: %.3f, rl: %.3d, \r\n", uwb_aoa_pkg->distance, uwb_aoa_pkg->angle, uwb_aoa_pkg->rssi_len);

				break;
			}

			case NOTIFY_DISTANCE_ANGLE_RSSI_FOBID:
			{
				uwb_aoa_fob_pkg_t *uwb_aoa_fob_pkg = (uwb_aoa_fob_pkg_t *)&radar_rx_Buf[2];
				*buffer = uwb_aoa_fob_pkg;

				UWB_LOG("[UWB]: dis: %.3f, agl: %.3f, pitch: %.3f\r\n", uwb_aoa_fob_pkg->distance, uwb_aoa_fob_pkg->angle, uwb_aoa_fob_pkg->pitch);
				ret = NOTIFY_DISTANCE_ANGLE_RSSI_FOBID;
				break;
			}

			default:
			break;
		}
		receive_tlv_seq = 0;
		rx_packet_ok = 0;
	}

	return ret;
}
