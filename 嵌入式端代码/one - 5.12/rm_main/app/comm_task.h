#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "stm32f4xx_hal.h"

#define CHASSIS_CAN_TX_ID 	0x200
#define GIMBAL_CAN_TX_ID  	0x1ff

#define USART_SEND_PERIOD	 2

#define GIMBAL_MOTOR_MSG_SEND     ( 1 << 6 )
#define CHASSIS_MOTOR_MSG_SEND    ( 1 << 7 )
#define SHOOT_MOTOR_MSG_SEND      ( 1 << 8 )

/* motor current parameter structure */
typedef struct
{
  /* 4 chassis motor current */
  int16_t chassis_cur[4];
  /* yaw/pitch motor current */
  int16_t gimbal_cur[2];
	int16_t shoot_cur[2];
} motor_current_t;

typedef __packed struct
{
	uint8_t  SOF;	//Ö¡Í·
	uint8_t  pit_angle_error[4];	//PITÖá½Ç¶È²î
	__packed struct
	{
		uint8_t  vacancy:1;		//¿ÕÏÐÎ»
		uint8_t  camp 	: 1;	//ÕóÓª
		uint8_t  aiming_status : 3;	//Ãé×¼Ä£Ê½
		uint8_t  shooter_speed : 3;	//ÉäËÙ
	}mode_msg;
	uint8_t  EOF1;	//Ö¡Î²1
	uint8_t  EOF2;	//Ö¡Î²2
}vision_tx_msg_t;

void can_msg_send_task(void const *argu);
void usart_msg_send_task(void const *argu);

extern motor_current_t motor_cur;
#endif
