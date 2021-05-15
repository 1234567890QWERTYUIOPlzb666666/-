#ifndef __BSP_TRIIGERMOTOR_H
#define __BSP_TRIIGERMOTOR_H

#include "pid.h"
#include "main.h"
#include "bsp_can.h"

#define TriiggerMotor_Ecd  32768  //拨盘一颗子弹转过的编码值
#define MIN_HEAT						10
void TriggerMotor_init(void);
void TriggerMotor_control(void);
void TriggerMotor_pidcal(void);
void ShootParam_Update(void);
#endif




