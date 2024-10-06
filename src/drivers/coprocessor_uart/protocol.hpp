#pragma once

#include "string.h"
// #define COPROCESSOR_UART_DEBUG 1

#define MAX_RECV_BUF_LEN 200

namespace protocol{

#pragma pack(push, 1)
typedef struct
{
	uint8_t EngineState[6];
	uint8_t Ecode[6];
 	uint16_t ECU_RPM[6];
	uint32_t rpm_timestamp[6];

	uint8_t oil_pos1;
	uint8_t oil_pos2;
	uint8_t data_num;

	uint16_t Pwm_Input[4];
	int16_t Pos_cmd[4];
	int16_t POS_Sensor[4];
	uint32_t Time_MG[4];
}protocol_data;

typedef struct
{
	uint8_t CMD;
	uint8_t engine_num;

}protocol_data_send;
#pragma pack(pop)

enum ServoOrder{
	theta_left = 0,
	theta_right = 1,
};


uint16_t crc16_table[256]={
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040};

uint8_t crc8(uint8_t *puchMsg,uint8_t crc_len, uint8_t seed)
{
	uint8_t i,k,crc8 = seed;
	for(i = 0; i < crc_len; i++)
	{
		k = puchMsg[i] ^ crc8;
		crc8 = 0;
		if (k & 0x01) crc8 ^= 0x5e;
		if (k & 0x02) crc8 ^= 0xbc;
		if (k & 0x04) crc8 ^= 0x61;
		if (k & 0x08) crc8 ^= 0xc2;
		if (k & 0x10) crc8 ^= 0x9d;
		if (k & 0x20) crc8 ^= 0x23;
		if (k & 0x40) crc8 ^= 0x46;
		if (k & 0x80) crc8 ^= 0x8c;
	}
	return crc8;
}
uint16_t  crc_bate(uint16_t crc, uint8_t c)
{
    uint8_t lut = (crc^c)&0xff;
    return (crc>>8) ^ crc16_table[lut];
}
uint16_t crc16(unsigned char *message, unsigned int len)
{
    uint16_t crc16 = 0;
    while(len --)
    {
        crc16 = crc_bate(crc16,*message++);
    }
    return crc16;
}
uint8_t crc_test()
{
//	uint8_t cr_te[7] = {0xf6, 0x00, 0x00, 0x9C, 0x38, 0x0C, 0xAB};
	uint8_t cr_te[3] = {0x20, 0x08, 0};
	uint8_t cr8;

	cr8 = crc8(cr_te, 2, 0x00);

	return cr8;
}

class dataParser
{
private:
	/* data */
	uint8_t _rxDataByte;
	enum rxStatus{
		RX_FREE,
		RX_HEAD_1,
		RX_HEAD_2,
		RX_LENGTH,
		RX_PATH,
		RX_CMD,
		RX_CRC8,
		RX_DATA,
		RX_CHECK_CRC16_BYTE1
	};
	enum rxStatus rx_status{RX_FREE};
public:
	dataParser(/* args */) {}
	~dataParser() {}

	void dataPack(const protocol_data_send &pd) {
		uint16_t sum = 0;
		uint8_t crc_size = 0;
		data_size = 0;
		//帧头
		_data[data_size++] = 0xaa;
		_data[data_size++] = 0x55;
			//数据长度
		_data[data_size++] = sizeof(pd);
		//地址
		_data[data_size++] = 0x02;
		//命令集
		_data[data_size++] = 0x10;
		crc_size = data_size;

		//CRC8
		_data[data_size++] = crc8(_data, crc_size, 0);
		// memcpy(_data + data_size - 1, &pd, sizeof(pd));
		memcpy(_data + data_size, &pd, sizeof(pd));

		//CRC16
		// data_size += sizeof(pd) - 1;
		data_size += sizeof(pd);
		sum = crc16(_data, data_size );

		_data[data_size++] = (uint8_t)(sum);
		_data[data_size++] = (uint8_t)(sum>>8);
	}

	bool protocolParser(uint8_t *rx_data, uint16_t length)
	{
		uint16_t i=0;
		rx_status = rxStatus::RX_FREE;
		while(i < length){
			// _rxDataByte = rx_data[i];
			switch(rx_status)
			{
			case rxStatus::RX_FREE:
				if((rx_data[i] == 0xAA) && (rx_data[i+1] == 0x55))
				{
					// Buf_num = 0;
					rx_status = rxStatus::RX_LENGTH;//自由状态下接到0xAA 0x55认为开始
					// Rsys_RX_Buf[Buf_num++] = 0xA5;
					i = i+2;
				}
				else
				{
					i++;
				}
			break;
			case rxStatus::RX_LENGTH:
				if(i+4 >= length){
					protocol_s.error_num = 6;
					return false;
				}
				protocol_s.length = rx_data[i];
				protocol_s.path = rx_data[i+1];
				protocol_s.cmd = rx_data[i+2];
				if(crc8(rx_data + i - 2, 5, 0) == rx_data[i+3]){
					rx_status = rxStatus::RX_CHECK_CRC16_BYTE1;
					i = i + 4; //data point head
				}else {
					protocol_s.error_num = 1;
					protocol_s.cmd = crc8(rx_data + i - 2, 5, 0);
					protocol_s.path = rx_data[i+3];
					return false;
				}
			break;
			case rxStatus::RX_CHECK_CRC16_BYTE1:
				if((i + protocol_s.length + 2) > length){
					protocol_s.error_num = 4;
					return false;
				}
				if(crc16(rx_data, protocol_s.length + 6) == (rx_data[protocol_s.length + 6] | rx_data[protocol_s.length + 7]<<8))
				{
					protocol_s.error_num = 0;
					memcpy(&(protocol_s.pd), rx_data + 6, sizeof(protocol_s.pd));
					return true;
				}
				protocol_s.error_num = 7;
				return false;
			break;
			default:
			break;
			}

		}
		protocol_s.error_num = 5;
		return false;
	}

	char protocol_parser(uint8_t *readbuf, uint16_t &readbuf_index, uint16_t readbuf_len)
	{
		int temp=0;
		char ret = -1;
		//char *end;
		// data_parser_flag = false;

		switch (parser_state) {
		case rxStatus::RX_FREE:
			if (readbuf[readbuf_index] == 0xAA ) {
				parser_state = rxStatus::RX_HEAD_1;
				parse_buf[parse_buf_index] = readbuf[readbuf_index];
				parse_buf_index++;
				ret = 1;
			}

			break;

		case rxStatus::RX_HEAD_1:
			if (readbuf[readbuf_index] == 0x55 ) {
				parser_state = rxStatus::RX_HEAD_2;
				parse_buf[parse_buf_index] = readbuf[readbuf_index];
				parse_buf_index++;

			} else {
				parser_state = rxStatus::RX_FREE;
				parse_buf_index = 0;
			}
			ret = 2;
			break;

		case rxStatus::RX_HEAD_2:
			parser_state = rxStatus::RX_LENGTH;
			parse_buf[parse_buf_index] = readbuf[readbuf_index];
			protocol_s.length = readbuf[readbuf_index];
			parse_buf_index++;
			ret = 3;
			break;

		case rxStatus::RX_LENGTH:
			parser_state = rxStatus::RX_PATH;
			parse_buf[parse_buf_index] = readbuf[readbuf_index];
			protocol_s.path = readbuf[readbuf_index];
			parse_buf_index++;
			ret = 4;
			break;

		case rxStatus::RX_PATH:
			parser_state = rxStatus::RX_CMD;
			parse_buf[parse_buf_index] = readbuf[readbuf_index];
			protocol_s.cmd = readbuf[readbuf_index];
			parse_buf_index++;
			ret = 5;
			break;

		case rxStatus::RX_CMD:
			if(crc8(parse_buf, 5, 0) == readbuf[readbuf_index])
			{
				parser_state = rxStatus::RX_CRC8;
				parse_buf[parse_buf_index] = readbuf[readbuf_index];
				parse_buf_index++;
			}else
			{
				parser_state = rxStatus::RX_FREE;
				parse_buf_index = 0;
			}
			ret = 6;
			break;

		case rxStatus::RX_CRC8:
			parser_state = rxStatus::RX_DATA;
			temp = math::min(readbuf_len - readbuf_index , (int)protocol_s.length);
			memcpy(parse_buf + 6, readbuf + readbuf_index, temp );
			parse_buf_index += temp;
			readbuf_index += temp-1;
			ret = 7;
			break;

		case rxStatus::RX_DATA:
			if(protocol_s.length + 6 > parse_buf_index){
				temp =  math::min(readbuf_len - readbuf_index , protocol_s.length + 6 - parse_buf_index);
				memcpy(&(parse_buf[parse_buf_index]), readbuf, temp);
				if( (readbuf_len - readbuf_index) >= (protocol_s.length + 6 - parse_buf_index) ){
					parser_state = rxStatus::RX_CHECK_CRC16_BYTE1;
					// parse_buf_index += temp;
					// readbuf_index += temp-1;
					parse_buf[protocol_s.length + 6] = readbuf[readbuf_index + temp];
				}
				parse_buf_index += temp+1;
				readbuf_index += temp;

				ret = 8;
			}else if(protocol_s.length + 6 < parse_buf_index){
				parser_state = rxStatus::RX_FREE;
				parse_buf_index = 0;
				ret = 9;
			}else{
				parser_state = rxStatus::RX_CHECK_CRC16_BYTE1;
				parse_buf[parse_buf_index] = readbuf[readbuf_index];
				parse_buf_index++;
				ret = 10;
			}

			break;

		case rxStatus::RX_CHECK_CRC16_BYTE1:
			parser_state = rxStatus::RX_FREE;
			parse_buf[parse_buf_index] = readbuf[readbuf_index];
			ret = 11;
			if(crc16(parse_buf, protocol_s.length + 6) == (parse_buf[protocol_s.length + 6] | parse_buf[protocol_s.length + 7]<<8))
			{
				memcpy(&(protocol_s.pd), parse_buf + 6, sizeof(protocol_s.pd));
				ret = 0;
				parser_num++;
				data_parser_flag = true;
			}
			parse_buf_index = 0;

			break;
		}
		#ifdef COPROCESSOR_UART_DEBUG
		// printf("state: TFMINI_PARSE_STATE %d ret: %d parse_buf_index: %d readbuf_len: %d readbuf_index: %d\n ", parser_state,ret,parse_buf_index,readbuf_len,readbuf_index);
		#endif
		readbuf_index++;
		return ret;
	}


	struct protocolStruct
	{
		uint8_t path;
		uint8_t cmd;
		uint8_t length;
		uint8_t error_num;
		protocol_data pd;
	};
	struct protocolStruct protocol_s;
	protocol_data_send cmd_send;

	uint8_t _data[MAX_RECV_BUF_LEN];
	uint8_t parse_buf[MAX_RECV_BUF_LEN];
	uint16_t parse_buf_index = 0;
	char parser_status = 0;
	bool data_parser_flag = false;
	uint32_t parser_num = 0;
	uint8_t data_size;

	rxStatus parser_state = rxStatus::RX_FREE;
};


}
