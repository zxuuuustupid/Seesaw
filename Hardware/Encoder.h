#ifndef ENCODER_H_
#define ENCODER_H_
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <Motor.h>
#include <Encoder.h>
typedef struct _Encoder_TypeDef
{                                   // 写一个结构体，用于存放编码器测量数据的信�??
    uint32_t CounterSettingValue;   // MCU的，TIM输入捕获通道，计数�?�的设置上限
    uint32_t OverFlowTimes;         // 暂时没用
    uint32_t CurrEncoderCountValue; // 当前的编码器脉冲数周期的计数值
    uint32_t LastEncoderCountValue; // 上一次的编码器脉冲数周期的计数值
    uint32_t WatchDogTick;          // 设置1个看门狗计数，如果超时，说明轮子已经不转了，所以脉冲数长时间没有更新

    float MotorDir;          // 电机的转动方�??
    float PeriodTimeSec;     // 编码器脉冲数计数值的，读取周期，相当于是转�?�测量周�??
    float Ratio;             // 轮子转动1周的编码器脉冲数，这�??1个系数�?�，�?? 电机的减速比�??82�?? * 电机1转输出脉冲数�??9个）  计算得到
    float RotationSpeed;     // 当前的转速（即，转�?�的实时测量值）
    float LastRotationSpeed; // 上一次测得的转�??
} EncoderTypeDef;
void EncoderInit(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
#endif /* ENCODER_H_ */
