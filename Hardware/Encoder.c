#include "Encoder.h"
EncoderTypeDef MotorEncoder[2];
int32_t Encoder_Interral[2] = {0};
float Num_Rounds = 0.0;
float Distance_Measure = 0.0;
void EncoderInit(void)
{ // 编码器测量的初始�??
    MotorEncoder[0].CounterSettingValue = 50000;
    MotorEncoder[0].CurrEncoderCountValue = 0;
    MotorEncoder[0].LastEncoderCountValue = 0;
    MotorEncoder[0].OverFlowTimes = 0;
    MotorEncoder[0].PeriodTimeSec = 0.01;
    MotorEncoder[0].Ratio = 82.0 * 9.0;
    MotorEncoder[0].RotationSpeed = 0.0;
    MotorEncoder[0].WatchDogTick = 0;

    MotorEncoder[1].CounterSettingValue = 50000;
    MotorEncoder[1].CurrEncoderCountValue = 0;
    MotorEncoder[1].LastEncoderCountValue = 0;
    MotorEncoder[1].OverFlowTimes = 0;
    MotorEncoder[1].PeriodTimeSec = 0.01;
    MotorEncoder[1].Ratio = 82.0 * 9.0;
    MotorEncoder[1].RotationSpeed = 0.0;
    MotorEncoder[1].WatchDogTick = 0;

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

    HAL_TIM_Base_Start_IT(&htim5); // �??1个TIM5的定时中断，专门用于转�?�测量的看门狗，在IDE那边设置的是0.01s中断
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t tempvalue1 = 0;
    uint32_t tempvalue2 = 0;
    if (htim == &htim3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {                                     // 触发了TIM3_CH1的输入捕获中断
            MotorEncoder[0].WatchDogTick = 0; // 说明有脉冲信号，电机没有停转，看门狗计数清0
            // 读出来当前的TIM3_CH1计数值，这个值是，上一次的脉冲上升沿，到，这一次的脉冲上升沿，之间的时钟计数值
            // 按本例子的IDE设置，相当于是，以 80MHz/800 =10uS为1个TICK的，对编码器进行 脉冲周期计算
            // 比如，计了100个TICK，就说明脉冲周期是100*10uS=1mS
            tempvalue1 = (uint32_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            MotorEncoder[0].LastEncoderCountValue = MotorEncoder[0].CurrEncoderCountValue; // 记录下来上一次的计数值
            MotorEncoder[0].CurrEncoderCountValue = tempvalue1;                            // 更新这一次（当前）的计数值

            // 如果上一次的脉冲计数值LastEncoderCountValue，大于了，这一次（当前）的计数值CurrEncoderCountValue
            // 说明 上一次测量到这一次（当前）测量的中途，出现了TIM3_CH1计数值溢出，也就是超出计数上限CounterSettingValue设置的50000，从0开始重新计数了
            // 比如，上一次读到的计数值是49000，这一次只读到100，说明实际上应该是累积计到了50100
            if (MotorEncoder[0].CurrEncoderCountValue < MotorEncoder[0].LastEncoderCountValue)
            {
                // 换算成当前的转速（即，转速的实时测量值）
                MotorEncoder[0].RotationSpeed = 60 * 100000 / (float)(MotorEncoder[0].CurrEncoderCountValue + MotorEncoder[0].CounterSettingValue - MotorEncoder[0].LastEncoderCountValue) / MotorEncoder[0].Ratio;
            }
            else
            {
                MotorEncoder[0].RotationSpeed = 60 * 100000 / (float)(MotorEncoder[0].CurrEncoderCountValue - MotorEncoder[0].LastEncoderCountValue) / MotorEncoder[0].Ratio;
            }
            MotorEncoder[0].RotationSpeed = MotorEncoder[0].MotorDir * MotorEncoder[0].RotationSpeed;
            if (MotorEncoder[0].MotorDir == 1.0f)
            {
                Encoder_Interral[0]--;
            }
            else if (MotorEncoder[0].MotorDir == -1.0f)
            {
                Encoder_Interral[0]++;
            }
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        MotorEncoder[1].WatchDogTick = 0;
        tempvalue2 = (uint32_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        MotorEncoder[1].LastEncoderCountValue = MotorEncoder[1].CurrEncoderCountValue;
        MotorEncoder[1].CurrEncoderCountValue = tempvalue2;
        if (MotorEncoder[1].CurrEncoderCountValue < MotorEncoder[1].LastEncoderCountValue)
        {
            MotorEncoder[1].RotationSpeed = 60 * 100000 / (float)(MotorEncoder[1].CurrEncoderCountValue + MotorEncoder[1].CounterSettingValue - MotorEncoder[1].LastEncoderCountValue) / MotorEncoder[1].Ratio;
        }
        else
        {
            MotorEncoder[1].RotationSpeed = 60 * 100000 / (float)(MotorEncoder[1].CurrEncoderCountValue - MotorEncoder[1].LastEncoderCountValue) / MotorEncoder[1].Ratio;
        }
        MotorEncoder[1].RotationSpeed = MotorEncoder[1].MotorDir * MotorEncoder[1].RotationSpeed;
        if (MotorEncoder[1].MotorDir == 1.0f)
        {
            Encoder_Interral[1]--;
        }
        else if (MotorEncoder[1].MotorDir == -1.0f)
        {
            Encoder_Interral[1]++;
        }
    }

    Num_Rounds = (float)(Encoder_Interral[1] + Encoder_Interral[0]) / 2.0f / 82.0f / 9.0f; // 计算电机转过的圈数
    Distance_Measure = (float)(Num_Rounds * 2.0f * 3.1415926f * 0.066f * 0.47f);           // 计算电机转过的距离
}
