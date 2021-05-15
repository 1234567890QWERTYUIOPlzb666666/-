#include "comm_task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "pid.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "string.h"
#include "bsp_can.h"
#include "bsp_judge.h"
#include "bsp_T_imu.h"
#include "control_def.h"

motor_current_t motor_cur;
vision_tx_msg_t		vision_tx_msg = {0};

/**
  * @brief can_msg_send_task
  * @param 
  * @attention  
  * @note  
  */
	
void can_msg_send_task(void const *argu)
{
	osEvent event;
	for(;;)
  {
		event = osSignalWait(GIMBAL_MOTOR_MSG_SEND  | \
		                     CHASSIS_MOTOR_MSG_SEND | \
		                     SHOOT_MOTOR_MSG_SEND , osWaitForever);
		if (event.status == osEventSignal)
    {
			if(ctrl_mode ==PROTECT_MODE || !lock_flag)
			{
				for(int i=0;i<4;i++)		motor_cur.chassis_cur[i]=0;
				for(int i=0;i<2;i++)		motor_cur.gimbal_cur[i]=0;
				for(int i=0;i<2;i++)		motor_cur.shoot_cur[i]=0;
				can1_send_message(GIMBAL_CAN_TX_ID, 0,0,0,0);
				can2_send_message(GIMBAL_CAN_TX_ID, 0,0,0,0);
				can1_send_message(CHASSIS_CAN_TX_ID,0,0,0,0);
			}
			else if(lock_flag && imu_data.wz!=0)
			{
				if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
				{
					can1_send_message(GIMBAL_CAN_TX_ID, -motor_cur.gimbal_cur[0],0,0,0);
					can2_send_message(GIMBAL_CAN_TX_ID, 0,-motor_cur.gimbal_cur[1],motor_cur.shoot_cur[0],motor_cur.shoot_cur[1]);
				}
				if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
				{
					can1_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur[0],motor_cur.chassis_cur[1],motor_cur.chassis_cur[2],motor_cur.chassis_cur[3]);
				}
				can1_send_supercap();
			}
		}
	}
}



/**
  * @brief usart_msg_send_task
  * @param 
  * @attention  
  * @note  
  */
float pit_angle_error_buf = 0;
void usart_msg_send_task(void const *argu)
{
	
	uint8_t vision_tx_buf[8];
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();

		/* 帧头 */
		vision_tx_msg.SOF = 0x11;
		/* 获取PIT与水平角度差 */
		if(Robot_Number==4)
		{
			pit_angle_error_buf = ((moto_pit.ecd - gimbal.pit_center_offset) / (-22.752f));
		}
		else if(Robot_Number==3)
		{
			pit_angle_error_buf = 0.463f*((moto_pit.ecd - gimbal.pit_center_offset) / (-22.752f));
		}
		memcpy(vision_tx_msg.pit_angle_error, &pit_angle_error_buf, 4);
		/* 获取射速信息 */
		vision_tx_msg.mode_msg.shooter_speed = shoot.shoot_speed_vision;
		/* (反陀螺/吊射)模式开关 */
		keyboard_scanf(rc.kb.bit.R, KB_R);
		keyboard_scanf(rc.kb.bit.G, KB_G);
		if(kb_status[KB_R] == -1)
			vision_tx_msg.mode_msg.aiming_status = 1;	//反陀螺
		else if(kb_status[KB_G] == -1)
			vision_tx_msg.mode_msg.aiming_status = 4;	//吊射
		else
			vision_tx_msg.mode_msg.aiming_status = 0;	//自瞄
		if(Game_Robot_Status.robot_id>100)		vision_tx_msg.mode_msg.camp=1;
		else 																	vision_tx_msg.mode_msg.camp=0;
		/* 帧尾 */
		vision_tx_msg.EOF1 = 0x22;
		vision_tx_msg.EOF2 = 0x33;

		/* 打包视觉发送信息 */
		memcpy(vision_tx_buf, &vision_tx_msg, 8);
		HAL_UART_Transmit_DMA(&huart6, vision_tx_buf, 8);

		taskEXIT_CRITICAL();
		osDelayUntil(&mode_wake_time, USART_SEND_PERIOD);
	}
}
