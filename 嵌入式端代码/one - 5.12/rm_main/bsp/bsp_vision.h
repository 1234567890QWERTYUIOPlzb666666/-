/** 
  * @file bsp_vision.h
  * @version 1.0
  * @date Feb,23th 2021
	*
  * @brief  视觉信息解算
	*
  *	@author 
  *
  */
#ifndef	__BSP_VISION_H__
#define __BSP_VISION_H__

#include "stdint.h"

typedef struct 
{
	float angle_error[2];	//角度误差
	float aim_speed;			//敌方相对速度
	float abs_speed;			//敌方绝对速度
	float predict;				//自瞄预测量
	struct
	{
		float angle_error;
		float aim_speed;
		float abs_speed;
		float imu_speed;
	} kal;								//卡尔曼滤波相关
} vision_gimbal_t;

typedef struct 
{
	vision_gimbal_t  pit; //PIT轴相关视觉信息
	vision_gimbal_t  yaw;	//YAW轴相关视觉信息
	
	float  		distance;			//敌方距离
	float     tof;					//子弹飞行时间
	uint16_t	period;				//视觉运算周期
	char  		cnt;					//自加位(保证视觉信息在持续变化)
	char  		eof;					//帧尾
	uint8_t   aim_flag;			//索敌标志位 不丢目标置1
} vision_msg_t;

extern vision_msg_t vision;

void vision_data_handler(uint8_t *Vision_Data);

#endif

