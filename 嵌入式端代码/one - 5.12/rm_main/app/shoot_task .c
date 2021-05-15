#include "shoot_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "usart.h"
#include "bsp_judge.h"
#include "control_def.h"
#include "pid.h"
#include "bsp_vision.h"

shoot_mode_t shoot;					//枪管模式 0为双枪管  1为下枪管 2为上枪管
shoot_t shoot_upper;				//上枪管结构体
shoot_t shoot_under;				//下枪管结构体
#define abs(x)		((x>0)? (x): (-x)) 

void shoot_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		shoot_mode_sw();							//射击模式切换
		FricMotor_Control();					//摩擦轮电机控制
		TriggerMotor_control();				//拨弹电机控制
		
		osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
	}
}

void shoot_init()
{
	FricMotor_init();
	shoot.shoot_mode        = SHOOT_STOP_MODE;
  shoot_upper.shoot_mode  = SHOOT_STOP_MODE;
	shoot_under.shoot_mode  = SHOOT_STOP_MODE;
	shoot.shoot_frequency   = SHOOT_FREQUENCY;
	shoot_upper.cooling_rate=10;		
	shoot_upper.heat_max=50;
	shoot.shoot_speed=15;
	TriggerMotor_init();
}

void shoot_mode_sw()
{
	static int KB_Q_last;
	keyboard_scanf(rc.kb.bit.Q, KB_Q);	//Q键双枪管切枪 
	if(KB_Q_last != kb_status[KB_Q])		shoot.barrel_mode++; //枪管模式 0为双枪管  1为下枪管 2为上枪管
	if(shoot.barrel_mode>2)		shoot.barrel_mode=0;
	KB_Q_last=kb_status[KB_Q];
	
	if(shoot.shoot_speed==15)					shoot.shoot_speed_vision=3;
	else if(shoot.shoot_speed==18)		shoot.shoot_speed_vision=4;
	else if(shoot.shoot_speed==30)		shoot.shoot_speed_vision=5;		//视觉射速档位处理
	
	if(ctrl_mode==PROTECT_MODE)
	{
		LASER_DOWN;	
		shoot.shoot_mode = SHOOT_STOP_MODE;
		shoot.firc_mode = FRIC_STOP_MODE;
	}
	else if(ctrl_mode==KEYBOARD_MODE || ctrl_mode==VISION_MODE || ctrl_mode==ENERGY_MODE)
	{
//		if(rc.mouse.l == 1)		shoot.shoot_mode = SHOOT_FIRE_MODE;
//		else									shoot.shoot_mode = SHOOT_STOP_MODE;
		 if(vision.yaw.angle_error[1]<=0&&vision.yaw.angle_error[1]>-5&&vision.distance!=0&&abs(vision.pit.angle_error[1])<=45&&rc.sw2==RC_DN) shoot.shoot_mode = SHOOT_FIRE_MODE;
		   else                                shoot.shoot_mode = SHOOT_STOP_MODE;
	}
	
	if(shoot.shoot_mode == SHOOT_FIRE_MODE)
	{
		if(shoot.barrel_mode == DOUBLE_BARREL_MODE)				//双枪管模式
		{
			shoot_upper.shoot_mode = SHOOT_FIRE_MODE;
			shoot_under.shoot_mode = SHOOT_FIRE_MODE;
			shoot.shoot_frequency=SHOOT_FREQUENCY;
		}
		else if(shoot.barrel_mode == UNDER_BARREL_MODE)		//下枪管模式
		{
			shoot_upper.shoot_mode = SHOOT_STOP_MODE;
			shoot_under.shoot_mode = SHOOT_FIRE_MODE;
			shoot.shoot_frequency=SHOOT_FREQUENCY*2;
		}
		else if(shoot.barrel_mode == UPPER_BARREL_MODE)		//上枪管模式
		{
			shoot_upper.shoot_mode = SHOOT_FIRE_MODE;
			shoot_under.shoot_mode = SHOOT_STOP_MODE;
			shoot.shoot_frequency=SHOOT_FREQUENCY*2;
		}
	}
	else
	{
		shoot_upper.shoot_mode = SHOOT_STOP_MODE;			
		shoot_under.shoot_mode = SHOOT_STOP_MODE;	
	}	
}
