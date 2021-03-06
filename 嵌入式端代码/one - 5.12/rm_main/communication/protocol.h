#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

//RM协议内置命令码   //发送的ID号
typedef enum
{
  CHASSIS_ODOM_CMD_ID = 0x0301,
  CHASSIS_CTRL_CMD_ID = 0x0302,
  GIMBAL_CTRL_ID = 0x0303,
	GAME_INFO_CMD_ID = 0x0304,
} referee_data_cmd_id_tpye;

//RM协议帧头结构体
typedef  struct
{
  uint8_t SOF;//帧头
  uint16_t data_length;
  uint8_t seq;//数据包ID
  uint8_t CRC8;
} frame_header_struct_t;

//RM协议反序列化步骤枚举
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

//RM协议反序列化结构体
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

typedef struct
{
	
	uint8_t end1;
	uint8_t end2;
	
} taurus_end_info ;

/******************************/

//底盘速度控制信息包
typedef struct
{
  float vx;
  float vy;
  float vw;
	_Bool spin;
//	float pitch_angle;
//	float yaw_angle;
//	float distant;//距离
	//uint8_t spinstatus;
	
}  chassis_ctrl_info_t;

//里程计反馈数据
typedef struct
{
	
  float vx;
  float vy;
  float vw;
	float yaw;
//	uint8_t order[20];
}  chassis_odom_info_t;


#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
