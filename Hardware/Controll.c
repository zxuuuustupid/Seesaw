#include "Controll.h"
extern float pitch_angle;
Pitch_PIDTypeDef Pit;
Yaw_PIDTypeDef Yaw;
extern float yaw_Init;
float Pit_PID_Calculate(Pitch_PIDTypeDef *pidTypeDef, float PitchMeasure)
{
    pidTypeDef->PitchMeasure = PitchMeasure;

    pidTypeDef->PitchLastError = pidTypeDef->PitchError;
    pidTypeDef->RPMLastOutput = pidTypeDef->RPMOutput;

    pidTypeDef->PitchError = pidTypeDef->PitchSetting - pidTypeDef->PitchMeasure;

    pidTypeDef->KPOut = pidTypeDef->KP * pidTypeDef->PitchError;
    pidTypeDef->KIOut += pidTypeDef->KI * pidTypeDef->PitchError;
    pidTypeDef->KDOut = pidTypeDef->KD * (pidTypeDef->PitchError - pidTypeDef->PitchLastError);

    if (pidTypeDef->KIOut > pidTypeDef->IntegralLimit)
    {
        pidTypeDef->KIOut = pidTypeDef->IntegralLimit;
    }
    else if (pidTypeDef->KIOut < -pidTypeDef->IntegralLimit)
    {
        pidTypeDef->KIOut = -pidTypeDef->IntegralLimit;
    }

    pidTypeDef->RPMOutput = pidTypeDef->KPOut + pidTypeDef->KDOut + pidTypeDef->KIOut;
    pidTypeDef->RPMOutput = 0.7f * pidTypeDef->RPMOutput + 0.3f * pidTypeDef->RPMLastOutput;
    if (pidTypeDef->RPMOutput > pidTypeDef->MaxRPMOutput)
    {
        pidTypeDef->RPMOutput = pidTypeDef->MaxRPMOutput;
    }
    else if (pidTypeDef->RPMOutput < -pidTypeDef->MaxRPMOutput)
    {
        pidTypeDef->RPMOutput = -pidTypeDef->MaxRPMOutput;
    }

    return pidTypeDef->RPMOutput;
}
void Pitch_PID_Init(void)
{
    Pit.PitchSetting = 0.0f;
    Pit.ControlPeriodSec = 0.01f;
    Pit.KP = 0.01f;
    Pit.KI = 0.0001f;
    Pit.KD = 0.001f;

    Pit.IntegralLimit = 50.0f;
    Pit.MaxRPMOutput = 50.0f;
    Pit.PitchMeasure = 0.0f;
    Pit.PitchLastError = 0.0f;
    Pit.RPMLastOutput = 0.0f;
    Pit.PitchError = 0.0f;
    Pit.KPOut = 0.0f;
    Pit.KIOut = 0.0f;
    Pit.KDOut = 0.0f;
    Pit.RPMOutput = 0.0f;
    Pit.FunctionCalPID = Pit_PID_Calculate;
}
void Yaw_PID_Init(void)
{
    Yaw.YawSetting = yaw_Init;
    Yaw.ControlPeriodSec = 0.01f;
    Yaw.KP = 10.0f;
    Yaw.KI = 0.0001f;
    Yaw.KD = 0.001f;

    Yaw.IntegralLimit = 50.0f;
    Yaw.MaxYawOutput = 50.0f;
    Yaw.YawMeasure = 0.0f;
    Yaw.YawLastError = 0.0f;
    Yaw.YawLastOutput = 0.0f;
    Yaw.YawError = 0.0f;
    Yaw.KPOut = 0.0f;
    Yaw.KIOut = 0.0f;
    Yaw.KDOut = 0.0f;
    Yaw.YawOutput = 0.0f;
    Yaw.FunctionCalPID = Yaw_PID_Calculate;
}
float Yaw_PID_Calculate(Yaw_PIDTypeDef *YawTypeDef, float YawMeasure)
{
    YawTypeDef->YawMeasure = YawMeasure;

    YawTypeDef->YawLastError = YawTypeDef->YawError;
    YawTypeDef->YawLastOutput = YawTypeDef->YawOutput;

    YawTypeDef->YawError = YawTypeDef->YawSetting - YawTypeDef->YawMeasure;

    YawTypeDef->KPOut = YawTypeDef->KP * YawTypeDef->YawError;
    YawTypeDef->KIOut += YawTypeDef->KI * YawTypeDef->YawError;
    YawTypeDef->KDOut = YawTypeDef->KD * (YawTypeDef->YawError - YawTypeDef->YawLastError);

    if (YawTypeDef->KIOut > YawTypeDef->IntegralLimit)
    {
        YawTypeDef->KIOut = YawTypeDef->IntegralLimit;
    }
    else if (YawTypeDef->KIOut < -YawTypeDef->IntegralLimit)
    {
        YawTypeDef->KIOut = -YawTypeDef->IntegralLimit;
    }

    YawTypeDef->YawOutput = YawTypeDef->KPOut + YawTypeDef->KDOut + YawTypeDef->KIOut;
    YawTypeDef->YawOutput = 0.7f * YawTypeDef->YawOutput + 0.3f * YawTypeDef->YawLastOutput;
    if (YawTypeDef->YawOutput > YawTypeDef->MaxYawOutput)
    {
        YawTypeDef->YawOutput = YawTypeDef->MaxYawOutput;
    }
    else if (YawTypeDef->YawOutput < -YawTypeDef->MaxYawOutput)
    {
        YawTypeDef->YawOutput = -YawTypeDef->MaxYawOutput;
    }

    return YawTypeDef->YawOutput;
}

void LED_Flash(int times)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(times);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(times);
}
