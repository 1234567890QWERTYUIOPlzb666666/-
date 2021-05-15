#include "bsp_TriggerMotor.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "shoot_task.h"
#include "math_calcu.h"
#include "string.h"
#include "bsp_judge.h"

extern TaskHandle_t can_msg_send_task_t;

void TriggerMotor_init(void)
{
	PID_struct_init(&pid_trigger_spd, POSITION_PID, 10000, 8000,
									5.0f,0.1f,0.0f); 
	PID_struct_init(&pid_trigger_ecd, POSITION_PID, 4320, 200,
									0.3f,0.0f,0.0f); 
	
	PID_struct_init(&pid_trigger2_spd, POSITION_PID, 10000, 8000,
									5.0f,0.1f,0.0f); 
	PID_struct_init(&pid_trigger2_ecd, POSITION_PID, 4320, 200,
									0.3f,0.0f,0.0f); 
}

void TriggerMotor_control(void)
{
	static uint16_t frequency_cnt=0;	//射频计算 
	ShootParam_Update();
	/*----------------------摩擦轮停转或保护模式时拨盘不转------------------------*/
	if(ctrl_mode==PROTECT_MODE || shoot.firc_mode == FRIC_STOP_MODE)
	{
		frequency_cnt=0;
		pid_trigger_spd.iout=0;
		pid_trigger2_spd.iout=0;
		shoot_upper.pid.trigger_ecd_ref=shoot_upper.pid.trigger_ecd_fdb;
		shoot_under.pid.trigger_ecd_ref=shoot_under.pid.trigger_ecd_fdb;
	}
	
	else if(ctrl_mode!=PROTECT_MODE && shoot.shoot_mode == SHOOT_FIRE_MODE)
	{
		shoot_upper.pid.trigger_ecd_error=shoot_upper.pid.trigger_ecd_ref-shoot_upper.pid.trigger_ecd_fdb;
		shoot_under.pid.trigger_ecd_error=shoot_under.pid.trigger_ecd_ref-shoot_under.pid.trigger_ecd_fdb;
		
		if(shoot_upper.shoot_mode && frequency_cnt==0)
		{
			if(ABS(shoot_upper.pid.trigger_ecd_error)< 2*TriiggerMotor_Ecd && shoot_upper.heat_remain >= MIN_HEAT)
			{
				shoot_upper.pid.trigger_ecd_ref += TriiggerMotor_Ecd;
				shoot_upper.heat+=10;
			}
		}
		if(shoot_under.shoot_mode && frequency_cnt==500/shoot.shoot_frequency)	//1000/SHOOT_PERIOD/shoot_frequency
		{
			if(ABS(shoot_under.pid.trigger_ecd_error)< 2*TriiggerMotor_Ecd && shoot_under.heat_remain >= MIN_HEAT)
			{
				shoot_under.pid.trigger_ecd_ref -= TriiggerMotor_Ecd;
				shoot_under.heat+=10;
			}
		}
		frequency_cnt++;
		if(frequency_cnt>=1000/shoot.shoot_frequency)	frequency_cnt=0;	
	}
	TriggerMotor_pidcal();
	osSignalSet(can_msg_send_task_t, SHOOT_MOTOR_MSG_SEND);
}

void ShootParam_Update(void)
{
	/*----------------------发射器裁判系统数据更新------------------------*/
	if(Game_Robot_Status.shooter_id1_17mm_speed_limit!=0)
	{
		shoot.shoot_speed=Game_Robot_Status.shooter_id1_17mm_speed_limit;
		shoot_upper.heat_max=Game_Robot_Status.shooter_id1_17mm_cooling_limit;
		shoot_upper.cooling_rate=Game_Robot_Status.shooter_id1_17mm_cooling_rate;
		
		shoot_under.heat_max=Game_Robot_Status.shooter_id1_17mm_cooling_limit;
		shoot_under.cooling_rate=Game_Robot_Status.shooter_id1_17mm_cooling_rate;
	}
	
	shoot_upper.heat-=shoot_upper.cooling_rate*SHOOT_PERIOD*0.001;
	if(shoot_upper.heat<0)	shoot_upper.heat=0;
	shoot_upper.heat_remain=shoot_upper.heat_max-shoot_upper.heat;
	
	shoot_under.heat-=shoot_under.cooling_rate*SHOOT_PERIOD*0.001;
	if(shoot_under.heat<0)	shoot_under.heat=0;
	shoot_under.heat_remain=shoot_under.heat_max-shoot_under.heat;
}

void TriggerMotor_pidcal(void)
{
	/*------------------------上枪管拨盘串级pid计算------------------------*/
	shoot_upper.pid.trigger_ecd_fdb = motor_trigger2.total_ecd;
	pid_calc(&pid_trigger_ecd, shoot_upper.pid.trigger_ecd_fdb,shoot_upper.pid.trigger_ecd_ref);
	shoot_upper.pid.trigger_spd_ref = pid_trigger_ecd.pos_out;		//位置环
	
	shoot_upper.pid.trigger_spd_fdb = motor_trigger2.speed_rpm;
	pid_calc(&pid_trigger_spd, shoot_upper.pid.trigger_spd_fdb,shoot_upper.pid.trigger_spd_ref);
	shoot_upper.current = pid_trigger_spd.pos_out;								//速度环
				
	/*------------------------下枪管拨盘串级pid计算------------------------*/
	shoot_under.pid.trigger_ecd_fdb = motor_trigger.total_ecd;
	pid_calc(&pid_trigger2_ecd, shoot_under.pid.trigger_ecd_fdb,shoot_under.pid.trigger_ecd_ref);
	shoot_under.pid.trigger_spd_ref = pid_trigger2_ecd.pos_out;		//位置环
	
	shoot_under.pid.trigger_spd_fdb = motor_trigger.speed_rpm;
	pid_calc(&pid_trigger2_spd, shoot_under.pid.trigger_spd_fdb,shoot_under.pid.trigger_spd_ref);
	shoot_under.current = pid_trigger2_spd.pos_out;								//速度环
	
	motor_cur.shoot_cur[0]=shoot_under.current;
	motor_cur.shoot_cur[1]=shoot_upper.current;
}

