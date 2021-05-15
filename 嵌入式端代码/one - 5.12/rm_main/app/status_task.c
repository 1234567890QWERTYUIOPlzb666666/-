#include "status_task.h"
#include "tim.h"
#include "DataScope_DP.h"
#include "usart.h"
status_t status;
/**
  * @brief status_task
  * @param     
  * @attention  
	* @note  
  */
void status_task(void const *argu)
{
	for(;;)
	{
		osDelay(1000);
	}
}

void status_init()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);
}

void status_restore()
{
	status.dbus_status = 0;
	status.gyro_status = 0;
	for(int i = 0;i<4;i++)
	{	
	  status.chassis_status[i] = 0;
	}
	status.gimbal_status[0] = 0;
	status.gimbal_status[1] = 0;
	status.gimbal_status[2] = 0;
}
