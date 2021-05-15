#include "bsp_powerlimit.h"
#include "chassis_task.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math_calcu.h"
#include "math.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_can.h"
#include "can.h"
#include "modeswitch_task.h"
#include "pid.h"
#include "bsp_judge.h"
#include "control_def.h"
extern e_control_order control_order;
powercontrol_t powercontrol = {0};
supercap_t supercap;

/***************************************************************************/
/*功率控制要调的参数  最大功率   最小缓冲能量*/
uint8_t   MAX_POWER_JUDGE = 80;			//裁判系统给的机器人最大功率 该变量用于给初值  接收到裁判系统时会按裁判系统信息更新
#define 	BUFFER_MIN				30.0f		//预测缓冲能量小于该值则进行电流分配
/***************************************************************************/

extern CAN_TxHeaderTypeDef Tx1Message;		
extern CAN_RxHeaderTypeDef Rx1Message;

void Power_Control(int16_t * current)
{
	static uint8_t powerlimit_cnt=0;			//限制计数位	每次缓冲能量低于5后限制100ms 即限制50次功率
	if(supercap.mode!=2 || supercap.volage < 14.0f)			//超级电容非放电模式下才进行功率控制
	{
		if(powercontrol.power_buffer < BUFFER_MIN)	
		{		
			powercontrol.status=1;			//缓冲能量小时限制状态位 置1
			powerlimit_cnt = 0;
			powercontrol.limit_temp= powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);	//计算限制比例
		}
	
		if(powercontrol.status)
		{
			powerlimit_cnt++;
			for(uint8_t i=0;i<4;i++)
			{
				current[i] /= (ABS(powercontrol.limit_temp)+1.0f) ;
			}	
		}
	
		if(powerlimit_cnt >= POWERLIMIT_CNT)
		{
			powercontrol.status = 0;		//限制状态位清零
			powerlimit_cnt = 0;					//限制计数位清零
		}
	}
}

void SuperCap_Control(void)			//超级电容控制函数
{
	SuperCap_Mode_Update();			//超级电容模式更新函数
	if(supercap.mode==0)				//超级电容保护模式
	{
		supercap.charge_power_ref=0;
		supercap.charge_current_set=0;
	}
	else if(supercap.mode==1)		//超级电容只充电模式
	{
		supercap.charge_power_fdb=powercontrol.supply_power-powercontrol.chassis_power-5.0f;	//超级电容板空载5W功率
		if(supercap.charge_power_fdb<0)		supercap.charge_power_fdb=0;
		/*反馈值是电源功率-底盘功率  超级电容放电下 可利用的空闲功率=裁判系统最大功率*/
		supercap.charge_power_ref=powercontrol.max_power-powercontrol.chassis_power-10.0f;//充电功率要略低于理想充电功率
		if(supercap.charge_power_ref<0)	supercap.charge_power_ref=0;
		/*目标值是最大功率-底盘功率  就是可利用的充电功率  超级电容控制板充电电流=空闲功率/超级电容电压*/
		supercap.charge_current_set = supercap.charge_power_ref/(supercap.volage);
		if(supercap.charge_current_set<=0)				supercap.charge_current_set=0;
		else if(supercap.charge_current_set>=10)	supercap.charge_current_set=10;
	}
	else if(supercap.mode==2)		//超级电容边充电边放电模式
	{
		supercap.charge_power_ref=powercontrol.max_power-10;							//充电功率要略低于理想充电功率
		if(supercap.charge_power_ref<0)	supercap.charge_power_ref=0;
		supercap.charge_current_set = supercap.charge_power_ref/(supercap.volage);
		if(supercap.charge_current_set<=0)				supercap.charge_current_set=0;
		else if(supercap.charge_current_set>=10)	supercap.charge_current_set=10;
	}
}

void SuperCap_Mode_Update(void)
{
	if(!lock_flag || ctrl_mode ==PROTECT_MODE)
	{		
		supercap.mode=0;	//机器人未解锁，超级电容进入保护模式
	}
	else if(rc.kb.bit.SHIFT) 
	{	
		if(supercap.volage >= SUPERCAP_DISCHAGER_VOLAGE)				
			supercap.mode=2;		//容组压降大于10V，可进行充放电操作
		else if(powercontrol.power_buffer <= BUFFER_MIN)		
			supercap.mode=0;		//底盘要进行功率限制，停止充电
		else 	
			supercap.mode=1;		//放电过程中最低电压小于12V，且底盘用的功率不大时充电
	}
	else if(!rc.kb.bit.SHIFT)
	{
		if(powercontrol.power_buffer<=BUFFER_MIN)		
			supercap.mode=0;		//底盘要进行功率限制，停止充电
		else 
			supercap.mode=1;		//放电过程中最低电压小于12V，且底盘用的功率不大时充电
	}
}

void PowerParam_Update(void)
{
	/* 模拟裁判系统数据更新 */
	powercontrol.judge_power=Power_Heat_Data.chassis_power;
	powercontrol.judge_power_buffer=Power_Heat_Data.chassis_power_buffer;
	powercontrol.max_power=Game_Robot_Status.chassis_power_limit;
	powercontrol.power_buffer -= ( powercontrol.supply_power - powercontrol.max_power + POWER_OFFSET) * 0.002f;
	if(powercontrol.power_buffer>=60)	 powercontrol.power_buffer = 60;		//缓冲能量更新
	
	/* 底盘最大轮速刷新 */
	if(rc.kb.bit.SHIFT && supercap.volage > 14.0f)
	{
		chassis.wheel_max = SPEED_SUPERCAP;
		chassis.keyboard_input = 50.0f;
	}
	else if(powercontrol.max_power == 45)
	{
		chassis.wheel_max = SPEED_45W;
		chassis.keyboard_input = 35.0f;
	}
	else if(powercontrol.max_power == 50)
	{
		chassis.wheel_max = SPEED_50W;
		chassis.keyboard_input = 38.0f;
	}
	else if(powercontrol.max_power == 55)
	{
		chassis.wheel_max = SPEED_55W;
		chassis.keyboard_input = 41.0f;
	}
	else if(powercontrol.max_power == 60)
	{
		chassis.wheel_max = SPEED_60W;
		chassis.keyboard_input = 45.0f;
	}
	else if(powercontrol.max_power == 80)
	{
		chassis.wheel_max = SPEED_80W;				
		chassis.keyboard_input = 50.0f;
	}
	else if(powercontrol.max_power == 100)
	{
		chassis.wheel_max = SPEED_100W;				
		chassis.keyboard_input = 50.0f;
	}
	else if(powercontrol.max_power == 120)
	{
		chassis.wheel_max = SPEED_120W;				
		chassis.keyboard_input = 50.0f;
	}
}

void PowerControl_Init(void)	
{
	powercontrol.max_power = MAX_POWER_JUDGE;
	powercontrol.limit_kp = 2.0f;
	chassis.keyboard_input = 50.0f;
	powercontrol.power_buffer = 60.0f;
}

void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data)
{
	uint8_t buf;
	switch(can_id)
  {
		case CAN_SUPPLY_POWER_ID:
		{
			memcpy(&supercap.volage,CAN_Rx_data,4);
			memcpy(&powercontrol.supply_power,(CAN_Rx_data+4),4); 
			supercap.volume_percent=(supercap.volage-SUPERCAP_DISCHAGER_VOLAGE)/(SUPERCAP_MAX_VOLAGE-SUPERCAP_DISCHAGER_VOLAGE)*100;
			break;
		}
		case CAN_CHASSIS_POWER_ID:
		{
		//	memcpy(&powercontrol.chassis_power,CAN_Rx_data,4); 
		//	memcpy(&powercontrol.chassis_power,CAN_Rx_data,2);
      //   	powercontrol.chassis_power /=100;		
//			powercontrol.cnt = CAN_Rx_data[4];
			powercontrol.chassis_power = CAN_Rx_data[0]<<8|CAN_Rx_data[1];
			powercontrol.chassis_power = powercontrol.chassis_power /100;
			Game_Robot_Status.shooter_id1_17mm_cooling_limit = CAN_Rx_data[2];
			  Game_Robot_Status.robot_id =CAN_Rx_data[3]>>7;
		    buf = CAN_Rx_data[3]<<1;
			Game_Robot_Status.shooter_id1_17mm_speed_limit =buf>>1;
		
			Game_Robot_Status.shooter_id1_17mm_cooling_rate =CAN_Rx_data[4];
			Game_Robot_Status.chassis_power_limit = CAN_Rx_data[5];
			control_order.order1 = CAN_Rx_data[6];
			control_order.order2 = CAN_Rx_data[7];
			//Power_Heat_Data.shooter_id1_17mm_cooling_heat = CAN_Rx_data[2]<<8|CAN_Rx_data[3];
		//	Game_Robot_Status.shooter_id1_17mm_cooling_limit = CAN_Rx_data[4]<<8|CAN_Rx_data[5];
			//Game_Robot_Status.shooter_id1_17mm_cooling_rate = CAN_Rx_data[6]<<8|CAN_Rx_data[7];
			break;
		}
	}
}

void can1_send_supercap()
{
	uint8_t supercap_msg_uint8_t[8]={0};
	uint8_t FreeTxNum = 0; 
	
	Tx1Message.StdId = 0x010;
	Tx1Message.IDE 	 = CAN_ID_STD;
	Tx1Message.RTR   = CAN_RTR_DATA;
  Tx1Message.DLC   = 0x08;
	
	supercap_msg_uint8_t[0] = supercap.mode;		//超级电容模式
	memcpy(supercap_msg_uint8_t+1,&supercap.charge_current_set,sizeof(supercap.charge_current_set));
	//查询发送邮箱是否为空
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0) 
	{  
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
  }
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,supercap_msg_uint8_t,(uint32_t*)CAN_TX_MAILBOX1);
}
