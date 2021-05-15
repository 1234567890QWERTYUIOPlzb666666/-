#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stm32f4xx_hal.h"
#include "shoot_task.h"


#define Init_PWM	 900    

typedef enum
{
  FRIC_STOP_MODE 	 = 0x00,
  FRIC_FIRE_MODE   = 0x01,
} fric_mode_e;

void FricMotor_init(void);
void FricMotor_Control(void);
void FricGunControl(uint16_t pwm1,uint16_t pwm2);
#endif
