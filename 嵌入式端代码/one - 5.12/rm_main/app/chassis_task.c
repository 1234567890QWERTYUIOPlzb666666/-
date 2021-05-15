#include "chassis_task.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "string.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "math_calcu.h"
#include "math.h"
#include "control_def.h"
#include "bsp_powerlimit.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "protocol.h"
#include "referee.h"
#include "bsp_T_imu.h"
extern TaskHandle_t can_msg_send_task_t;
e_control_order control_order;
chassis_t chassis;
float odom_yaw;
float imu_aim_yaw;
float ecd_aim_yaw;
float imu_init_yaw;
float ecd_init_yaw;
float yaw_circle_set;
chassis_ctrl_info_t chassis_ctrl;
chassis_odom_info_t chassis_odom; 
/**
  * @brief chassis_task
  * @param     
  * @attention  
	* @note  
  */
void chassis_task(void const *argu)
{
	uint8_t spin_flag=0;
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();
		switch(ctrl_mode)
		{
			case PROTECT_MODE:
			case ENERGY_MODE:
			{
				chassis.spd_input.vx = 0;
				chassis.spd_input.vy = 0;
				chassis.spd_input.vw = 0;
				chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191); 
				chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
				//yaw_circle_set = moto_yaw.ecd;
					chassis.spd_real.vx = (+moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f *M3508_MOTOR_RPM_TO_VECTOR;
	      chassis.spd_real.vy = (+moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f *M3508_MOTOR_RPM_TO_VECTOR;
	      chassis.spd_real.vw = (-moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f *M3508_MOTOR_RPM_TO_VECTOR; 
				break;
			}
		
			case VISION_MODE:
			case KEYBOARD_MODE:
			{
				chassis_ramp();
				chassis.position_ref = gimbal.yaw_center_offset;
			//	chassis.position_ref = yaw_circle_set;
        ecd_aim_yaw = (moto_yaw.ecd -ecd_init_yaw)*(2.0*PI/8191.0f)*57.3;
				imu_aim_yaw = imu_data.yaw -imu_init_yaw;
				chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191); 
				chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
				chassis.spd_input.vx = chassis_ctrl.vx/M3508_MOTOR_RPM_TO_VECTOR;
				chassis.spd_input.vy = chassis_ctrl.vy/M3508_MOTOR_RPM_TO_VECTOR;
				chassis.spd_input.vw = chassis_ctrl.vw/M3508_MOTOR_RPM_TO_VECTOR*0.39f;
					if(rc.ch5==0)	spin_flag=1;
				if(rc.ch5 == 660 && spin_flag)   			
				{					 
					chassis.spin_status=~chassis.spin_status;
					spin_flag=0;
				}
				if(chassis.spin_status==-1)		//遥控器模式拨轮左拨开小陀螺 再左拨关
				{
					if(chassis.wheel_max<=6000)	
						chassis.spd_input.vw=chassis.wheel_max;
					else          chassis.spd_input.vw = chassis.wheel_max;		
				}
				else if(chassis.spin_status==0)
				{
					chassis.spd_input.vw = 0.0f;
				}
			//	chassis.spd_input.vx = 1.0f *(float)(chassis_x_ramp.out * cos(chassis.angle_error) + (-1.0f)*chassis_y_ramp.out * sin(chassis.angle_error));
			//	chassis.spd_input.vy = 1.0f *(float)(chassis_x_ramp.out * sin(chassis.angle_error) - (-1.0f)*chassis_y_ramp.out * cos(chassis.angle_error));
				//if(kb_status[KB_F] == -1)			chassis.spin_status=-1;		//键盘模式按F开启小陀螺
//				else													chassis.spin_status=0;
//				if(chassis.spin_status==-1)
//				{
//					if(chassis.wheel_max<=6000)		
//						chassis.spd_input.vw=chassis.wheel_max;
//					else          chassis.spd_input.vw = 6000;		
//				}
//				else if(chassis.spin_status==0)
//				{
//					chassis.spd_input.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref,chassis.position_ref + chassis.position_error);
//				}
				break;
			}		
			
			case REMOTER_MODE:
			{
				chassis.position_ref = gimbal.yaw_center_offset;
			//	yaw_circle_set =moto_yaw.ecd;
				chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191); 
				chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
  			chassis.spd_input.vx = 1.0f*(float)(rc.ch2*rc_ch2_scale * cos(chassis.angle_error) + (-1.0f)*rc.ch1*rc_ch1_scale * sin(chassis.angle_error));
		  	chassis.spd_input.vy = 1.0f*(float)(rc.ch2*rc_ch2_scale * sin(chassis.angle_error) - (-1.0f)*rc.ch1*rc_ch1_scale * cos(chassis.angle_error));
				if(rc.ch5==0)	spin_flag=1;
				if(rc.ch5 == 660 && spin_flag)   			
				{					 
					chassis.spin_status=~chassis.spin_status;
					spin_flag=0;
				}
				if(chassis.spin_status==-1)		//遥控器模式拨轮左拨开小陀螺 再左拨关
				{
					if(chassis.wheel_max<=6000)	
						chassis.spd_input.vw=chassis.wheel_max;
					else          chassis.spd_input.vw = chassis.wheel_max;		
				}
				else if(chassis.spin_status==0)
				{
					chassis.spd_input.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref,chassis.position_ref + chassis.position_error);
				}
				break;
			}
			default:
			{
				break;
			}
		}
		chassis_spd_distribution();		//底盘速度分配
		for (int i = 0; i < 4; i++)
		{
			chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm; 
			chassis.current[i] = pid_calc(&pid_chassis_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
		}
		PowerParam_Update();
		SuperCap_Control();
		Power_Control(chassis.current);		
		
		static uint8_t debug_i;
		debug_i++;
		if(debug_i == 30)	
		{
			DataWave(&huart3);
			debug_i=0;
		}
		
		memcpy(motor_cur.chassis_cur,chassis.current, sizeof(chassis.current));
		osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);
		taskEXIT_CRITICAL();
		osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
	}
}

/**
	* @brief 麦轮解算函数
	* @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
	*        output: every wheel speed(rpm)
	* @note  1=FL 2=FR 3=BL 4=BR
	*/
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
	int16_t wheel_rpm[4];
	
	wheel_rpm[0] =  vx + vy - vw;
	wheel_rpm[1] = -vx + vy - vw;
	wheel_rpm[2] =  vx - vy - vw;
	wheel_rpm[3] = -vx - vy - vw;
	
	memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}

void chassis_init()
{
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_chassis_spd[i], POSITION_PID, 13000, 5000,
										10.0f,	0.0f,	0.0f	);  
	}
	PID_struct_init(&pid_chassis_angle, POSITION_PID, 6000, 500,
									8.0f,	0.0f, 0.0f	);  
}

void sparate_move(void)
{
	 if(ABS(chassis.position_error) <= 300 ) 
	 chassis.position_ref = moto_yaw.ecd;		
	 if(chassis.position_error > 300 )
	 {	
	 chassis.position_ref = gimbal.yaw_center_offset - 300;
	 if(chassis.position_ref > 8191)
	 chassis.position_ref = chassis.position_ref - 8191;
	 }
		if(chassis.position_error < -300 )
	 {	
	 chassis.position_ref = gimbal.yaw_center_offset + 300;
	 if(chassis.position_ref < 0)
	 chassis.position_ref = chassis.position_ref + 8191;
	 }
	 chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
}
	
/**
	* @brief          底盘速度分配函数，输入速度超过最大轮速时，将输入的速度按比例分配到三个轴向上
	* @author         
	* @param[in]      void
	* @retval         void
	*/
	
void chassis_spd_distribution(void)
{
	float  wheel_spd_input_buf[4];
	float  wheel_spd_total=0;				//总轮速
	float  distribution_temp=1.0f;	//限制比例
	//麦轮逆解算
	chassis.spd_fdb.vx = (+moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
	chassis.spd_fdb.vy = (+moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
	chassis.spd_fdb.vw = (-moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
	
	mecanum_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);
	for(int i=0;i<4;i++)
	{	
		wheel_spd_input_buf[i]=ABS(chassis.wheel_spd_input[i]);	
		wheel_spd_total += wheel_spd_input_buf[i];
	}

	if(wheel_spd_total > (chassis.wheel_max * 4.0f))								//判断最大速度是否超额
	{
		distribution_temp = wheel_spd_total / (chassis.wheel_max * 4.0f);		 //超额速度除最大速度得分配比例
	}
	for(int j=0;j<4;j++)
	{	
		chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j] / distribution_temp ;		//速度重分配
		chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j],8500,-8500);		//电机转速最高到8900
	}
	chassis.spd_ref.vx = (+chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] + chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
	chassis.spd_ref.vy = (+chassis.wheel_spd_ref[0] + chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
	chassis.spd_ref.vw = (-chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;	
	chassis.spd_real.vx = (+moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f *M3508_MOTOR_RPM_TO_VECTOR;
	chassis.spd_real.vy = (+moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f *M3508_MOTOR_RPM_TO_VECTOR;
	chassis.spd_real.vw = (-moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f *M3508_MOTOR_RPM_TO_VECTOR; 
	chassis.spd_error = chassis.spd_ref.vx + chassis.spd_ref.vy + chassis.spd_ref.vw 
										- chassis.spd_fdb.vx - chassis.spd_fdb.vy - chassis.spd_fdb.vw;		
	//计算目标速度和反馈速度差值，大于一定值用超级电容放电
}

/**
  * @brief          底盘斜坡启动函数，通过斜坡函数映射目标速度
  * @author         
  * @param[in]      void
  * @retval         void
  */
void chassis_ramp(void)
{
  if(rc.kb.bit.W)
	{
	  ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);			
	}
	else if(rc.kb.bit.S)
	{
		ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);			
	}
	else
	{
		if(chassis_x_ramp.out > 0)
		{
			ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input,chassis.wheel_max , 0.0f);			
		}
		else if(chassis_x_ramp.out < 0)
		{
			ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);	
		}
	}
	if(rc.kb.bit.D)
	{
	  ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);			
	}
	else if(rc.kb.bit.A)
	{
	  ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);			
	}
	else
	{
		if(chassis_y_ramp.out > 0)
		{
			ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, chassis.wheel_max, 0.0f);			
		}
		else if(chassis_y_ramp.out < 0)
		{
			ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);	
		}
	}	
}

/**
  * @brief          底盘S型启动函数，通过S型函数映射目标速度
  * @author         
  * @param[in]      void
  * @retval         void
  */
void chassis_sigmoid(void)
{
	static float x_speed_sigmoid,y_speed_sigmoid;
	static float x_input,y_input;
	static float x_sigmoid,y_sigmoid;
	if(rc.kb.bit.W) 					x_input += SIGMOID_PERIOD;
	else if(rc.kb.bit.S)			x_input -= SIGMOID_PERIOD;
	else
	{
		if(x_input > 0)					x_input -= SIGMOID_PERIOD;
		else if(x_input < 0)		x_input += SIGMOID_PERIOD;
	}
	if(rc.kb.bit.D) 					y_input += SIGMOID_PERIOD;
	else if(rc.kb.bit.A)			y_input -= SIGMOID_PERIOD;			
	else
	{
		if(y_input > 0)					y_input -= SIGMOID_PERIOD;		
		else if(y_input < 0)		y_input += SIGMOID_PERIOD;	
	}	
	
	if(x_input >= (2*SIGMOID_MAX))					x_input=  (2*SIGMOID_MAX);
	else if(x_input <= -(2*SIGMOID_MAX))		x_input= -(2*SIGMOID_MAX);
	 
	if(y_input >= (2*SIGMOID_MAX))					y_input=  (2*SIGMOID_MAX);
	else if(y_input <= -(2*SIGMOID_MAX))		y_input= -(2*SIGMOID_MAX);
	
	if(x_input <= ABS(SIGMOID_PERIOD) && x_input >= -ABS(SIGMOID_PERIOD))	
		x_sigmoid = 0;
	else if(x_input >= ABS(SIGMOID_PERIOD))												
		x_sigmoid = Sigmoid_function(x_input);
	else if(x_input <= -ABS(SIGMOID_PERIOD))
		x_sigmoid = -Sigmoid_function(x_input);

	if(y_input <= ABS(SIGMOID_PERIOD) && y_input >= -ABS(SIGMOID_PERIOD))	
		y_sigmoid = 0;
	else if(y_input >= ABS(SIGMOID_PERIOD))												
		y_sigmoid = Sigmoid_function(y_input);
	else if(y_input <= -ABS(SIGMOID_PERIOD))
		y_sigmoid = -Sigmoid_function(y_input);
	
	x_speed_sigmoid = x_sigmoid * chassis.wheel_max;
	y_speed_sigmoid = y_sigmoid * chassis.wheel_max;
	
	chassis.spd_input.vx = (float)(x_speed_sigmoid * cos(chassis.angle_error) + (-1.0f)*y_speed_sigmoid * sin(chassis.angle_error));
  chassis.spd_input.vy = (float)(x_speed_sigmoid * sin(chassis.angle_error) - (-1.0f)*y_speed_sigmoid * cos(chassis.angle_error));
}
void chassis_odom_send_task(void const * argument)
{
	
    while(1)
    {
			chassis_odom.vx =chassis.spd_real.vx;
			//chassis_odom.vx = vx;
			//chassis_odom.vx = chassis_ctrl.vx ;
			chassis_odom.vy = -chassis.spd_real.vy;
			//chassis_odom.vy = vy;
			//chassis_odom.vy = chassis_ctrl.vy;
		 chassis_odom.vw = chassis.spd_real.vw;
//			for(int i=0;i<20;i++)
//			{
//			      chassis_odom.order[i] = Comm_getdata.data[i+6];
//			}
			//chassis_odom.vw = vw;
		//	chassis_odom.vw = chassis_ctrl.vw;
	//		odom_yaw = imu_data.yaw/57.3f+chassis.position_error*0.043f/57.3f;
			odom_yaw = -circle_error(0,imu_data.yaw,360)+chassis.position_error*0.043f;
			  //odom_yaw = imu_aim_yaw -ecd_aim_yaw;
			//odom_yaw = chassis.position_error*0.043f/57.3f;
			chassis_odom.yaw = odom_yaw/57.3f;
			referee_send_data(CHASSIS_ODOM_CMD_ID, (void*)&chassis_odom, sizeof(chassis_odom));
			osDelay(10);
    }
}
