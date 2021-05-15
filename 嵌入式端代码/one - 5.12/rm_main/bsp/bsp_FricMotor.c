#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "control_def.h"

Slope_Struct shoot_Fric_pwm_upper;		//��ǹ��		//��ǹ�ܾ�����ǹ��
Slope_Struct shoot_Fric_pwm_under; 		//��ǹ��			

void FricMotor_init(void)
{ 
	//900-2000
	//����ʱ�����Ŵ����
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); 	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4); 
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1); 	
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2); 
	
  Upper_FricMotor_PWM = Init_PWM;
	Under_FricMotor_PWM = Init_PWM;		//�������ʼ����PWM
	
	shoot_Fric_pwm_upper.limit_target = Init_PWM;//��ʼ��Ħ����б�º���
	shoot_Fric_pwm_upper.real_target  = Init_PWM;
	shoot_Fric_pwm_upper.change_scale = 0.5;
	
	shoot_Fric_pwm_under.limit_target = Init_PWM;//��ʼ��Ħ����б�º���
	shoot_Fric_pwm_under.real_target  = Init_PWM;
	shoot_Fric_pwm_under.change_scale = 0.5;
			
	shoot.shoot_speed = 15;		//��ʼ������
}

/**
	*@func   		void FricGunControl(uint8_t Control)
	*@bref			Ħ������ͣ
	*@param[in] Control��0Ϊֹͣ��1Ϊ����
  *@retval    void
	*@note			900������ת 
	*/
void FricGunControl(uint16_t pwm1,uint16_t pwm2)
{
	shoot_Fric_pwm_upper.limit_target=Init_PWM+pwm1;
	shoot_Fric_pwm_under.limit_target=Init_PWM+pwm2;
	
	if(lock_flag)			//��������ܹ����PWM
	{
		Slope_On(&shoot_Fric_pwm_upper); //��Ħ����б������
		Slope_On(&shoot_Fric_pwm_under); //��Ħ����б������ 
		
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
			FricGunControl(0,0);		//Ħ����ͣת	
			break;
		}
		
		case FRIC_FIRE_MODE:
		{
			if(shoot.shoot_speed == 15)
			{
				FricGunControl(LOW_SPEED_UPPER,LOW_SPEED);		//��ǹ���ٶ���һЩ
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
