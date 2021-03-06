#define __GIMBAL_TASK_GLOBALS
#include "chassis_task.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "remote_msg.h"
#include "math_calcu.h"
#include "bsp_T_imu.h"
#include "bsp_vision.h"
#include "KalmanFilter.h"
#include "usart.h"

#define PIT_SEN  0.15f
#define YAW_SEN	 -0.005f

extern TaskHandle_t can_msg_send_task_t;
gimbal_t gimbal;
e_gimbal_mode gimbal_pit_mode;
e_gimbal_mode gimbal_pit_last_mode;
e_gimbal_mode gimbal_yaw_mode;
e_gimbal_mode gimbal_yaw_last_mode;
e_gimbal_mode auto_gimbal_mode;
e_gimbal_mode auto_gimbal_last_mode;
float test;
void gimbal_param_init(void)
{
  memset(&gimbal, 0, sizeof(gimbal_t));

  PID_struct_init(&pid_pit_ecd, POSITION_PID, 5000, 0,
                  pid_pit_ecd_P, pid_pit_ecd_I, pid_pit_ecd_D); 		
  PID_struct_init(&pid_pit_spd, POSITION_PID, 28000,20000,
                  pid_pit_spd_P, pid_pit_spd_I, pid_pit_spd_D);	
	
	PID_struct_init(&pid_yaw_angle, POSITION_PID, 5000, 0,
                  pid_yaw_angle_P, pid_yaw_angle_I, pid_yaw_angle_D);		
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 28000, 20000,
									pid_yaw_spd_P,pid_yaw_spd_I, pid_yaw_spd_D);			

	gimbal.pit_center_offset = gimbal_pit_center_offset;
	gimbal.yaw_center_offset = gimbal_yaw_center_offset;
	auto_gimbal_mode =stop_mode;
}


/**
  * @brief gimbal_task
  * @param     
  * @attention  
	* @note  
  */
void gimbal_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();
		switch(ctrl_mode)
		{
			case PROTECT_MODE:
			{
				gimbal.pid.pit_ecd_ref = gimbal_pit_center_offset;
				gimbal.pid.yaw_angle_ref = imu_data.yaw;
				gimbal_mode_update();
				for(int i=0;i<2;i++)	gimbal.current[i]=0;
				break;
			}	
			
			case REMOTER_MODE:
			{
				gimbal.pid.pit_ecd_ref  	+= rc.ch4 * rc_ch4_scale;		
				gimbal.pid.yaw_angle_ref  += rc.ch3 * rc_ch3_scale;		
        gimbal_mode_update();				
				gimbal_pid_calcu();
				break;
			}
			
			case KEYBOARD_MODE:
			{
				//gimbal.pid.pit_ecd_ref   += rc.mouse.y *  PIT_SEN;
				//gimbal.pid.yaw_angle_ref += rc.mouse.x *  YAW_SEN;
				gimbal_mode_update();
				gimbal_target_update();
				if(auto_gimbal_mode!=vision_mode)
				{
				  gimbal_pid_calcu();
				}
				break;
			}
			
			case VISION_MODE:
			{
				gimbal_mode_update();
				if(vision.distance)		//??????????????????????????????
				{
					vision_calcu();
					vision.aim_flag=1;
				}
				else if(vision.aim_flag==1)	//???????????????????????????  ??????????????? ???????????????
				{
					gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb;
					gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
					vision.aim_flag=0;
				}
				else
				{
					gimbal.pid.pit_ecd_ref   += rc.mouse.y * PIT_SEN;
					gimbal.pid.yaw_angle_ref += rc.mouse.x * YAW_SEN;
					gimbal_pid_calcu();
				}
				break;
			}	
			default:
			{
			}break;
		}		
		
		memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
		osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
		taskEXIT_CRITICAL();     
		
		osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
	}
}

void vision_calcu()
{
	/* pit???????????????????????? */
	if(ABS(vision.pit.angle_error[1]) <= 45)     //??????????????????????????????
	{
		vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_angle_error, vision.pit.angle_error[1] * 22.75f);
		gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb - vision.pit.kal.angle_error; //2*22.75
	}
	else						//????????????????????????????????????pit???
	{	
		gimbal.pid.pit_ecd_ref   += rc.mouse.y *  PIT_SEN;	
	}
	
	
	/* yaw???????????????????????? */	
	if(kb_status[KB_R] == -1)		//??????????????? ??????10??????????????????????????? ????????????  ????????????????????? ??????????????????
	{
		if(vision.yaw.angle_error[1]!=0)
			gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb - vision.yaw.angle_error[1];
	}
	else	//??????????????????
	{
//		if(vision.distance<=2000)
//		{
//			vision.yaw.predict = vision.yaw.kal.abs_speed	/8.0f;
//		}
//		else if(vision.distance>2000)
//		{
//			vision.yaw.predict = vision.yaw.kal.abs_speed	/5.0f;
//		}
//		vision.yaw.predict = vision.yaw.kal.abs_speed	/20.0f;
		if(vision.yaw.predict>=10.0f)					vision.yaw.predict=10.0f;
		else if(vision.yaw.predict<=-10.0f)		vision.yaw.predict=-10.0f;
//		if(vision.distance <= 1500)			vision.yaw.predict=0;
//		if(vision.yaw.angle_error[1] > 7)		vision.yaw.predict=0;
		vision.yaw.predict = vision.yaw.kal.abs_speed/20;
		if(vision.yaw.angle_error[1]>7)
		{vision.yaw.predict = 0;}
		if(vision.yaw.angle_error[1]>=10)
			 vision.yaw.angle_error[1]=10;
		if(vision.yaw.angle_error[1]<=-10)
			 vision.yaw.angle_error[1]=-10;
		vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_angle_error, vision.yaw.angle_error[1]-vision.yaw.predict*2.5);//
		gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb - vision.yaw.kal.angle_error;
	}
	
	gimbal_pid_calcu();
}

void gimbal_pid_calcu()			
{
/*------------------------pit?????????pid??????------------------------*/
	gimbal.pid.pit_ecd_ref  = data_limit(gimbal.pid.pit_ecd_ref,GIMBAL_PIT_MAX,GIMBAL_PIT_MIN);	//???????????????
	gimbal.pid.pit_ecd_fdb = moto_pit.ecd;
	gimbal.pid.pit_ecd_error = circle_error(gimbal.pid.pit_ecd_ref,gimbal.pid.pit_ecd_fdb,8191);
	pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_ecd_error);
	gimbal.pid.pit_spd_ref =  pid_pit_ecd.pos_out;   //PID???????????????
	gimbal.pid.pit_spd_fdb =  imu_data.wy;					 //pit?????????????????????PID?????????
	pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
	
/*------------------------yaw?????????pid??????------------------------*/
	if(gimbal.pid.yaw_angle_ref<0)            gimbal.pid.yaw_angle_ref += 360;	
	else if(gimbal.pid.yaw_angle_ref>360)     gimbal.pid.yaw_angle_ref -= 360;	//???????????????
	gimbal.pid.yaw_angle_fdb = imu_data.yaw;
	gimbal.pid.yaw_angle_error = circle_error(gimbal.pid.yaw_angle_ref,gimbal.pid.yaw_angle_fdb,360);
	pid_calc(&pid_yaw_angle, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_fdb + gimbal.pid.yaw_angle_error);
	gimbal.pid.yaw_spd_ref = pid_yaw_angle.pos_out;			
	gimbal.pid.yaw_spd_fdb = imu_data.wz;		
	pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);	
	
	gimbal.current[0] = -1.0f * pid_yaw_spd.pos_out;
	gimbal.current[1] = -1.0f * pid_pit_spd.pos_out;
}


void vision_energy_calcu()
{
/*------------------------pit????????????????????????------------------------*/		
	vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_energy, 0.55f*vision.pit.angle_error[1]);
	gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb - vision.pit.kal.angle_error;
	
/*------------------------yaw????????????????????????------------------------*/	
	if(ABS(vision.yaw.angle_error[1]) >= 55)		
		vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_energy, 0.03f*vision.yaw.angle_error[1]);	
	else if(ABS(vision.yaw.angle_error[1]) < 55)		
		vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_energy, 0.02f*vision.yaw.angle_error[1]);	
	gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb - vision.yaw.kal.angle_error;
	
	gimbal_pid_calcu();
}
void gimbal_mode_update()
{
	  auto_gimbal_last_mode = auto_gimbal_mode;
    if(gimbal.pid.pit_ecd_fdb>=(GIMBAL_PIT_AUT_MAX-30)&&ctrl_mode==KEYBOARD_MODE&&vision.distance==0)
				{
				   auto_gimbal_mode = up_mode;
				}
				if(gimbal.pid.pit_ecd_fdb<=GIMBAL_PIT_AUT_MIN&&ctrl_mode==KEYBOARD_MODE&&vision.distance==0)
				{
				   auto_gimbal_mode = down_mode;
				}
		if(ctrl_mode!=KEYBOARD_MODE)
		{
		      auto_gimbal_mode = stop_mode;
		}
		if(auto_gimbal_mode==stop_mode&&ctrl_mode==KEYBOARD_MODE)
		{
		      auto_gimbal_mode = up_mode;
		}
			if(auto_gimbal_last_mode==vision_mode&&ctrl_mode==KEYBOARD_MODE)
		{
		      auto_gimbal_mode = up_mode;
		}
		if(vision.distance&&ctrl_mode==KEYBOARD_MODE)
		{
		    auto_gimbal_mode = vision_mode;
			//  gimbal_yaw_mode = vision_mode;
	  }
				
}
void gimbal_target_update()
{
	 	if(auto_gimbal_mode==vision_mode)
	{
	     vision_calcu();
	}
	else if(auto_gimbal_last_mode==vision_mode&&auto_gimbal_mode!=vision_mode)
	{
	        
		      gimbal.pid.pit_ecd_ref   = gimbal.pid.pit_ecd_fdb;
					gimbal.pid.yaw_angle_ref = imu_data.yaw;
	}
    if(auto_gimbal_mode == down_mode)
				{
				    gimbal.pid.pit_ecd_ref += 2.0f;
					 gimbal.pid.yaw_angle_ref += 0.2f;
			
				}
				if(auto_gimbal_mode  == up_mode)
				{
				    gimbal.pid.pit_ecd_ref -= 2.0f;
					 gimbal.pid.yaw_angle_ref += 0.2f;
				
				}
		if(auto_gimbal_mode==stop_mode)
		{
		      gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb;
			    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
	
		}			 
				if(gimbal.pid.yaw_angle_ref>=360)
				{
				  gimbal.pid.yaw_angle_ref -=360.0f;
				}

}
