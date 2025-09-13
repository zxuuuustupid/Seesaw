#include "Motor.h"
PIDTypeDef MotorPID[2];
extern EncoderTypeDef MotorEncoder[2];
float PIDCalculate(PIDTypeDef *pid, float rpmMeasure)
{ // PID计算函数
  pid->RPMMeasure = rpmMeasure;

  pid->RPMLastError = pid->RPMError; // 先记录下来上1次的误差和PWM输出的设置值
  pid->PWMLastOutput = pid->PWMOutput;

  pid->RPMError = pid->RPMSetting - pid->RPMMeasure; // 计算出来转速误差值

  pid->KPOut = pid->KP * pid->RPMError;                       // 计算出来P项的输出�??
  pid->KDOut = pid->KD * (pid->RPMError - pid->RPMLastError); // 计算出来D项的输出�??
  pid->KIOut += pid->KI * pid->RPMError;                      // 计算出来I项的输出�??

  if (pid->KIOut > pid->IntegralLimit)
  { // 计算出来I项的输出值，要进行限制，不能大于IntegralLimit的�??
    // 因为这个值是�??直累积的，有爆�?�的可能性！！！
    pid->KIOut = pid->IntegralLimit;
  }
  if (pid->KIOut < -pid->IntegralLimit)
  {
    pid->KIOut = -pid->IntegralLimit;
  }

  pid->PWMOutput = pid->KPOut + pid->KIOut + pid->KDOut; // 计算出来总的PWM输出的设置�??

  pid->PWMOutput = 0.7f * pid->PWMOutput + 0.3f * pid->PWMLastOutput; // 为了让PWM输出平稳，不要一下子大，�??下子�??
                                                                      // 在一定程度上参�?�上�??次的PWM输出的设置�?�PWMLastOutput
                                                                      // 这里的权重系�?? 0.7   0.3 是可以自行调整的
  if (pid->PWMOutput > pid->MaxPWMOutput)
  { // �??大的PWM输出设置值，不能大于MCU�?? PWM通道输出设置�?? �??大计数�??
    pid->PWMOutput = pid->MaxPWMOutput;
  }
  if (pid->PWMOutput < -pid->MaxPWMOutput)
  {
    pid->PWMOutput = -pid->MaxPWMOutput;
  }

  return pid->PWMOutput;
}

void MotorPIDInit(void)
{                                // 初始化电机的PID控制，做好控制的准备
  MotorPID[0].RPMSetting = 0.0; // 设置电机的目标转�??
  if (MotorPID[0].RPMSetting > 0)
  { // 判断电机转动方向
    HAL_GPIO_WritePin(MotorDirCH1_GPIO_Port, MotorDirCH1_Pin, GPIO_PIN_SET);
    MotorEncoder[0].MotorDir = 1.0;
  }
  else
  {
    HAL_GPIO_WritePin(MotorDirCH1_GPIO_Port, MotorDirCH1_Pin, GPIO_PIN_RESET);
    MotorEncoder[0].MotorDir = -1.0;
  }

  MotorPID[0].ControlPeriodSec = 0.01;
  MotorPID[0].MaxPWMOutput = 500.0;
  MotorPID[0].IntegralLimit = 350.0;

  MotorPID[0].KP = 10.0;
  MotorPID[0].KD = 3.0;
  MotorPID[0].KI = 1.0;

  MotorPID[0].PWMOutput = 0;

  MotorPID[0].PosDeadBand = 150;
  MotorPID[0].NegDeadBand = -150;

  MotorPID[0].FunctionCalPID = PIDCalculate; // 挂载PID计算函数

  MotorPID[1].RPMSetting = 0.0;
  if (MotorPID[1].RPMSetting > 0)
  {
    HAL_GPIO_WritePin(MotorDirCH2_GPIO_Port, MotorDirCH2_Pin, GPIO_PIN_RESET);
    MotorEncoder[1].MotorDir = 1.0;
  }
  else
  {
    HAL_GPIO_WritePin(MotorDirCH2_GPIO_Port, MotorDirCH2_Pin, GPIO_PIN_SET);
    MotorEncoder[1].MotorDir = -1.0;
  }

  MotorPID[1].ControlPeriodSec = 0.01;
  MotorPID[1].MaxPWMOutput = 500.0;
  MotorPID[1].IntegralLimit = 350.0;

  MotorPID[1].KP = 10.0;
  MotorPID[1].KD = 3.0;
  MotorPID[1].KI = 1.0;

  MotorPID[1].PWMOutput = 0;

  MotorPID[1].PosDeadBand = 150;
  MotorPID[1].NegDeadBand = -150;

  MotorPID[1].FunctionCalPID = PIDCalculate;
}

void MotorCtrl(uint8_t chn, float setting)
{ // 产生电机控制�??�??的PWM输出
  switch (chn)
  {
  case 0:
    if (setting > 0)
    {
      if (MotorPID[0].RPMSetting > 0)
      {
        //                    HAL_GPIO_WritePin(MotorDirCH1_GPIO_Port, MotorDirCH1_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, setting + MotorPID[0].PosDeadBand);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      }
    }
    else
    {
      if (MotorPID[0].RPMSetting < 0)
      {
        //                    HAL_GPIO_WritePin(MotorDirCH1_GPIO_Port, MotorDirCH1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -1 * setting - MotorPID[0].NegDeadBand);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      }
    }
    //
    break;
  case 1:
    if (setting > 0)
    {
      if (MotorPID[1].RPMSetting > 0)
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, setting + MotorPID[1].PosDeadBand);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      }
    }
    else
    {
      if (MotorPID[1].RPMSetting < 0)
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -1 * setting - MotorPID[1].NegDeadBand);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      }
    }
    break;
  default:
    break;
  }
}
