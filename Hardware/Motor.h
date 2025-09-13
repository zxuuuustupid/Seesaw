#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Encoder.h"
#include "tim.h"
typedef struct _PID_TypeDef
{           // 写一个结构体，用于存放PID控制的信�??
  float KP; // PID设置�??
  float KI;
  float KD;

  float RPMSetting;   // 转�?�设置�??
  float RPMMeasure;   // 转�?�测量�??
  float RPMError;     // 转�?�误差�?�（当前�??
  float RPMLastError; // 上一次的  转�?�误差�??

  float PWMOutput;     // PWM输出的设置�?�（当前�??
  float PWMLastOutput; // 上一次的  PWM输出的设置�??

  float KPOut; // PID计算中，P项计算得到的输出�??float PIDCalculate(PIDTypeDef *pid, float rpmMeasure)
  float KIOut; // PID计算中，I项计算得到的输出�??
  float KDOut; // PID计算中，D项计算得到的输出�??

  float MaxPWMOutput;  // �??大的PWM输出设置值（不能大于MCU的，PWM通道输出设置的，�??大计数�?�）
  float IntegralLimit; // 用于  限制�?? PID计算中，I项计算得到的输出值，不能大于IntegralLimit的�??
                       // 因为这个值是�??直累积的，有爆�?�的可能性！！！

  float PosDeadBand; // 正向转动的死区�?�（电机�?? 这个PWM输出设置值以内，不转动），每个电机不�??样，要测�??
  float NegDeadBand; // 反向转动的死区�?�（电机�?? 这个PWM输出设置值以内，不转动），每个电机不�??样，要测�??

  float ControlPeriodSec; // 控制周期设置

  float (*FunctionCalPID)(struct _PID_TypeDef *pidTypeDef, float rpmMeasure); // pid计算的函数指�??
} PIDTypeDef;
float PIDCalculate(PIDTypeDef *pid, float rpmMeasure);
void MotorPIDInit(void);
void MotorCtrl(uint8_t chn, float setting);
#endif // __MOTOR_H__
