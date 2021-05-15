#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "supercap_task.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "visionfire_task.h"
#include "remote_msg.h"
#include "cmsis_os.h"
#include "bsp_vision.h"

ctrl_mode_e ctrl_mode;
/* 键盘按键状态标志位 */
int kb_status[11]={0};

uint8_t lock_flag=1;
void mode_switch_task(void const *argu)
{
	for(;;)
	{
		 can2_send(0x004,10,20,30,40,50);
		if(!lock_flag)	
		{			
			unlock_init();			//解锁操作
		}
		sw1_mode_handler();	
		sw2_mode_handler();
		osDelay(5);
	}
}

void sw1_mode_handler(void)			//由拨杆1决定的模式切换，主要是云台和底盘
{
	switch (rc.sw1)
  {
		case RC_UP:
		{
			ctrl_mode = REMOTER_MODE;
			
			if(rc.ch5 == -660) shoot.house_switch=~shoot.house_switch;
			if(shoot.house_switch==-1)	TIM3->CCR3=2000;
			else												TIM3->CCR3=1010;		//遥控器模式下拨轮右拨打开弹仓盖
			break;
		}
		case RC_MI:
		{
			ctrl_mode = PROTECT_MODE;
			break;
		}
		case RC_DN:
		{
			if(ctrl_mode != ENERGY_MODE || ctrl_mode != VISION_MODE)		
				ctrl_mode = KEYBOARD_MODE;				//非大能量和视觉模式下重置为键盘模式
			LASER_UP;
			keyboard_scanf(rc.kb.bit.F, KB_F);	//F键开小陀螺
			shoot.firc_mode = FRIC_FIRE_MODE;
//			keyboard_scanf(rc.kb.bit.V,KB_V);		//V键开关摩擦轮 
//			if(kb_status[KB_V]==-1)		
//			{
//				shoot.firc_mode = FRIC_FIRE_MODE;
//				LASER_UP;
//			}
//			else 							
//			{
//				//shoot.firc_mode = FRIC_STOP_MODE;
//			//	LASER_DOWN;
//			}
			
//			keyboard_scanf(rc.kb.bit.C, KB_C);		//C键开打哨兵模式
//			if(kb_status[KB_C] == -1)		vision.predict_mode=1;
//			else  											vision.predict_mode=0;
			
			keyboard_scanf(rc.kb.bit.G, KB_G);	//G键大能量 
			
			keyboard_scanf(rc.kb.bit.B, KB_B);	//B键开弹仓盖
			if(kb_status[KB_B] == -1)			TIM3->CCR3=2000;	//1000为最低
			else													TIM3->CCR3=1010;	//2000为最高
			
			if(rc.mouse.r == 1)		  //右键开视觉		
			{
				if(kb_status[KB_G] == -1)		ctrl_mode = ENERGY_MODE;
				else 												ctrl_mode = VISION_MODE;	
			}
			else 		ctrl_mode = KEYBOARD_MODE;	
			
			break;
		}
		default:
		break;
  }
}

void sw2_mode_handler(void)		//由拨杆2决定的模式切换，主要是发射器
{ 
	if(ctrl_mode == REMOTER_MODE)
	{
		switch (rc.sw2)
		{
			case RC_UP:
			{
			//	LASER_DOWN;
				LASER_UP;
				shoot.firc_mode  = FRIC_STOP_MODE;
				shoot.shoot_mode = SHOOT_STOP_MODE;
				break;
			}
			case RC_MI:
			{
				LASER_UP
				shoot.firc_mode  = FRIC_FIRE_MODE;
				shoot.shoot_mode = SHOOT_STOP_MODE;
				break;
			}
			case RC_DN:
			{
				shoot.firc_mode  = FRIC_FIRE_MODE;
				shoot.shoot_mode = SHOOT_FIRE_MODE;
				break;
			}
		}
	}
}

void unlock_init(void)			//解锁函数
{
	if(rc.sw1 == RC_MI && rc.sw2 == RC_UP)
	{
	 if(rc.ch4==-660)
	 {
		if(rc.ch3==660)
		{
		 lock_flag = 1;
		}
	 }					
	}
}		
