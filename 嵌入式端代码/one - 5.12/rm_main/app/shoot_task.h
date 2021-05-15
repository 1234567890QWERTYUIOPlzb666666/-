#ifdef  __SHOOT_TASK_GLOBALS
#define __SHOOT_TASK_EXT
#else
#define __SHOOT_TASK_EXT extern
#endif

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "stm32f4xx_hal.h"
#include "bsp_FricMotor.h"

#define SHOOT_PERIOD 2

#define LASER_UP		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
#define LASER_DOWN	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);	//红外激光

typedef struct
{
	/* position loop ecd*/
  float trigger_ecd_ref;
  float trigger_ecd_fdb;
	float trigger_ecd_error;
  /* speed loop */
  float trigger_spd_ref;
  float trigger_spd_fdb;
} trigger_pid_t;

typedef struct
{
  uint8_t  shoot_mode;
	trigger_pid_t pid;
	int16_t  current;
	float   heat;
	int32_t   heat_remain;
	uint16_t  heat_max;			//最大热量	裁判系统读出
	uint16_t  cooling_rate;	//冷却速率  裁判系统读出
} shoot_t;

typedef struct
{
  uint8_t  shoot_mode;					//允许射击模式  右键开
	uint8_t  cover_mode;          //弹仓盖
	uint8_t  firc_mode;						//摩擦轮模式
  uint8_t  barrel_mode;					//枪管模式
	int      house_switch;				//弹仓盖
	uint16_t shoot_speed;					//射速
	uint8_t  shoot_frequency;	  	//射频
	uint8_t  shoot_speed_vision;	//发给视觉的射速档位
}shoot_mode_t;

typedef enum
{
	SHOOT_STOP_MODE,    
	SHOOT_FIRE_MODE,
} shoot_mode_e; 

typedef enum
{
	DOUBLE_BARREL_MODE,
	UNDER_BARREL_MODE,
	UPPER_BARREL_MODE,
} barrel_mode_e;

extern shoot_t shoot_upper;		//上枪管结构体
extern shoot_t shoot_under;		//下枪管结构体
extern shoot_mode_t shoot;
extern uint8_t shoot_frequency;


void shoot_task(void const *argu);
void shoot_init(void);
void shoot_mode_sw(void);

#endif
