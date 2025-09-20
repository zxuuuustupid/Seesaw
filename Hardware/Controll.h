#ifndef __CONTROLL_H_
#define __CONTROLL_H_
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <Motor.h>
#include <Encoder.h>
#include "FreeRTOS.h"
#include <math.h>                    // 添加这个头文件，包含sqrtf等数学函数的声明
#define M_PI 3.14159265358979323846f // 手动定义单精度π值
typedef struct _Pitch_PID_TypeDef
{             // 写一个结构体，用于存放PID控制的信�??
    float KP; // PID设置�??
    float KI;
    float KD;

    float PitchSetting;   // 俯仰设置�??
    float PitchMeasure;   // 俯仰测量�??
    float PitchError;     // 俯仰误差�?�（当前�??
    float PitchLastError; // 上一次的  俯仰误差�??

    float RPMOutput;     // PWM输出的设置�?�（当前�??
    float RPMLastOutput; // 上一次的  PWM输出的设置�??

    float KPOut; // PID计算中，P项计算得到的输出�??
    float KIOut; // PID计算中，I项计算得到的输出�??
    float KDOut; // PID计算中，D项计算得到的输出�??

    float MaxRPMOutput;  // �??大的PWM输出设置值（不能大于MCU的，PWM通道输出设置的，�??大计数�?�）
    float IntegralLimit; // 用于  限制�?? PID计算中，I项计算得到的输出值，不能大于IntegralLimit的�??

    float ControlPeriodSec; // 控制周期设置

    float (*FunctionCalPID)(struct _Pitch_PID_TypeDef *pidTypeDef, float PitchMeasure); // pid计算的函数指�??
} Pitch_PIDTypeDef;
typedef struct _Yaw_PID_TypeDef
{             // 写一个结构体，用于存放PID控制的信�??
    float KP; // PID设置�??
    float KI;
    float KD;

    float YawSetting;   // 偏航设置�??
    float YawMeasure;   // 偏航测量�??
    float YawError;     // 偏航误差�?�（当前�??
    float YawLastError; // 上一次的  偏航误差�??

    float YawOutput;     // PWM输出的设置�?�（当前�??
    float YawLastOutput; // 上一次的  PWM输出的设置�??

    float KPOut; // PID计算中，P项计算得到的输出�??
    float KIOut; // PID计算中，I项计算得到的输出�??
    float KDOut; // PID计算中，D项计算得到的输出�??

    float MaxYawOutput;  // �??大的PWM输出设置值（不能大于MCU的，PWM通道输出设置的，�??大计数�?�）
    float IntegralLimit; // 用于  限制�?? PID计算中，I项计算得到的输出值，不能大于IntegralLimit的�??

    float ControlPeriodSec; // 控制周期设置

    float (*FunctionCalPID)(struct _Yaw_PID_TypeDef *pidTypeDef, float YawMeasure); // pid计算的函数指�??
} Yaw_PIDTypeDef;
typedef struct _Distance_PID_TypeDef
{             // 写一个结构体，用于存放PID控制的信�??
    float KP; // PID设置�??
    float KI;
    float KD;

    float DistanceSetting;   // 距离设置�??
    float DistanceMeasure;   // 距离测量�??
    float DistanceError;     // 距离误差�?�（当前�??
    float DistanceLastError; // 上一次的  距离误差�??

    float DistanceOutput;     // PWM输出的设置�?�（当前�??
    float DistanceLastOutput; // 上一次的  PWM输出的设置�??

    float KPOut; // PID计算中，P项计算得到的输出�??
    float KIOut; // PID计算中，I项计算得到的输出�??
    float KDOut; // PID计算中，D项计算得到的输出�??

    float MaxDistanceOutput; // �??大的PWM输出设置值（不能大于MCU的，PWM通道输出设置的，�??大计数�?�）
    float IntegralLimit;     // 用于  限制�?? PID计算中，I项计算得到的输出值，不能大于IntegralLimit的�??

    float ControlPeriodSec; // 控制周期设置

    float (*FunctionCalPID)(struct _Distance_PID_TypeDef *pidTypeDef, float DistanceMeasure); // pid计算的函数指�??
} Distance_PIDTypeDef;

struct Places_Type
{
    float x;
    float y;
};
void Pitch_PID_Init(void);
float Pit_PID_Calculate(Pitch_PIDTypeDef *pidTypeDef, float PitchMeasure);
void Yaw_PID_Init(void);
float Yaw_PID_Calculate(Yaw_PIDTypeDef *pidTypeDef, float YawMeasure);
void LED_Flash(int times);
void Distance_PID_Init(void);
float Distance_PID_Calculate(Distance_PIDTypeDef *pidTypeDef, float DistanceMeasure);
void LED_Blink(int times);
void Xunhang(struct Places_Type places[4]);
void Yaw_Distance_Calculate(float *Yaw_Expect, float *Distance_Expect, float X_Expect, float Y_Expect, float X_Current, float Y_Current);
#endif
