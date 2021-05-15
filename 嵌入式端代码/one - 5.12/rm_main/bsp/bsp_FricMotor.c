#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "control_def.h"

Slope_Struct shoot_Fric_pwm_upper;		//上枪管		//单枪管就是上枪管
Slope_Struct shoot_Fric_pwm_under; 		//下枪管			

void FricMotor_init(void)
{ 
	//900-2000
	//启动时，油门打到最低
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); 	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4); 
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1); 	
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2); 
	
  Upper_FricMotor_PWM = Init_PWM;
	Under_FricMotor_PWM = Init_PWM;		//给电调初始化的PWM
	
	shoot_Fric_pwm_upper.limit_target = Init_PWM;//初始化摩擦轮斜坡函数
	shoot_Fric_pwm_upper.real_target  = Init_PWM;
	shoot_Fric_pwm_upper.change_scale = 0.5;
	
	shoot_Fric_pwm_under.limit_target = Init_PWM;//初始化摩擦轮斜坡函数
	shoot_Fric_pwm_under.real_target  = Init_PWM;
	shoot_Fric_pwm_under.change_scale = 0.5;
			
	shoot.shoot_speed = 15;		//初始化射速
}

/**
	*@func   		void FricGunControl(uint8_t Control)
	*@bref			摩擦轮起停
	*@param[in] Control：0为停止，1为启动
  *@retval    void
	*@note			900以上起转 
	*/
void FricGunControl(uint16_t pwm1,uint16_t pwm2)
{
	shoot_Fric_pwm_upper.limit_target=Init_PWM+pwm1;
	shoot_Fric_pwm_under.limit_target=Init_PWM+pwm2;
	
	if(lock_flag)			//解锁后才能够输出PWM
	{
		Slope_On(&shoot_Fric_pwm_upper); //上摩擦轮斜坡启动
		Slope_On(&shoot_Fric_pwm_under); //下摩擦轮斜坡启动 
		
		Upper_FricMotor_PWM = shoot_Fric_pwm_upper.real_target;
		Under_FricMotor_PWM = shoot_Fric_pwm_under.real_target;
	}

}     

void FricMotor_Control(void)
{
	switch (shoot.firc_mode)
	{
		case FRIC_STOP_MODE:
		{
			FricGunControl(0,0);		//摩擦轮停转	
			break;
		}
		
		case FRIC_FIRE_MODE:
		{
			if(shoot.shoot_speed == 15)
			{
				FricGunControl(LOW_SPEED_UPPER,LOW_SPEED);		//上枪管速度慢一些
			}	
			else if(shoot.shoot_speed == 18)
			{
				FricGunControl(MID_SPEED,MID_SPEED);		
			}
			else if(shoot.shoot_speed == 30)
			{
				FricGunControl(HIGH_SPEED,HIGH_SPEED);
			}
			break;
		}
		default:
		{
		}
	}
}
