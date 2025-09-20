#include "Controll.h"
extern float pitch_angle;
Pitch_PIDTypeDef Pit;
Yaw_PIDTypeDef Yaw;
Distance_PIDTypeDef Distance;
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
    Pit.KP = 10.0f;
    Pit.KI = 0.0001f;
    Pit.KD = 50.0f;

    Pit.IntegralLimit = 50.0f;
    Pit.MaxRPMOutput = 150.0f;
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
    Yaw.KP = 2.0f;
    Yaw.KI = 0.0001f;
    Yaw.KD = 2.00f;

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

// 距离PID初始化函数
void Distance_PID_Init(void)
{
    // 假设已经定义了全局的Distance_PIDTypeDef类型变量Distance
    Distance.DistanceSetting = 0.0f;   // 初始设定值，可根据实际需求修改
    Distance.ControlPeriodSec = 0.01f; // 控制周期，与Yaw保持一致为0.01秒

    // PID参数初始化，可根据实际调试需求修改
    Distance.KP = -200.0f;
    Distance.KI = 0.0001f;
    Distance.KD = 0.5f;

    Distance.IntegralLimit = 100.0f;     // 积分限幅
    Distance.MaxDistanceOutput = 100.0f; // 输出最大值限制

    // 状态变量初始化
    Distance.DistanceMeasure = 0.0f;
    Distance.DistanceLastError = 0.0f;
    Distance.DistanceLastOutput = 0.0f;
    Distance.DistanceError = 0.0f;
    Distance.KPOut = 0.0f;
    Distance.KIOut = 0.0f;
    Distance.KDOut = 0.0f;
    Distance.DistanceOutput = 0.0f;

    // 绑定PID计算函数
    Distance.FunctionCalPID = Distance_PID_Calculate;
}

// 距离PID计算函数
float Distance_PID_Calculate(Distance_PIDTypeDef *pidTypeDef, float DistanceMeasure)
{
    // 更新测量值
    pidTypeDef->DistanceMeasure = DistanceMeasure;

    // 保存上一次的误差和输出
    pidTypeDef->DistanceLastError = pidTypeDef->DistanceError;
    pidTypeDef->DistanceLastOutput = pidTypeDef->DistanceOutput;

    // 计算当前误差
    pidTypeDef->DistanceError = pidTypeDef->DistanceSetting - pidTypeDef->DistanceMeasure;

    // 计算PID各项输出
    pidTypeDef->KPOut = pidTypeDef->KP * pidTypeDef->DistanceError;
    pidTypeDef->KIOut += pidTypeDef->KI * pidTypeDef->DistanceError * pidTypeDef->ControlPeriodSec;                                  // 积分项乘以控制周期
    pidTypeDef->KDOut = pidTypeDef->KD * (pidTypeDef->DistanceError - pidTypeDef->DistanceLastError) / pidTypeDef->ControlPeriodSec; // 微分项除以控制周期

    // 积分限幅
    if (pidTypeDef->KIOut > pidTypeDef->IntegralLimit)
    {
        pidTypeDef->KIOut = pidTypeDef->IntegralLimit;
    }
    else if (pidTypeDef->KIOut < -pidTypeDef->IntegralLimit)
    {
        pidTypeDef->KIOut = -pidTypeDef->IntegralLimit;
    }

    // 计算总输出并加入一阶低通滤波
    pidTypeDef->DistanceOutput = pidTypeDef->KPOut + pidTypeDef->KDOut + pidTypeDef->KIOut;
    pidTypeDef->DistanceOutput = 0.7f * pidTypeDef->DistanceOutput + 0.3f * pidTypeDef->DistanceLastOutput;

    // 输出限幅
    if (pidTypeDef->DistanceOutput > pidTypeDef->MaxDistanceOutput)
    {
        pidTypeDef->DistanceOutput = pidTypeDef->MaxDistanceOutput;
    }
    else if (pidTypeDef->DistanceOutput < -pidTypeDef->MaxDistanceOutput)
    {
        pidTypeDef->DistanceOutput = -pidTypeDef->MaxDistanceOutput;
    }

    return pidTypeDef->DistanceOutput;
}

// 计算到达目标点所需的偏航角(±180度)和距离
void Yaw_Distance_Calculate(float *Yaw_Expect, float *Distance_Expect, float X_Expect, float Y_Expect, float X_Current, float Y_Current)
{
    // 检查指针有效性
    if (Yaw_Expect == NULL || Distance_Expect == NULL)
        return;

    // 计算坐标差值
    float delta_x = X_Expect - X_Current; // X方向差值
    float delta_y = Y_Expect - Y_Current; // Y方向差值

    // 计算当前位置到目标位置的直线距离（勾股定理）
    *Distance_Expect = sqrtf(delta_x * delta_x + delta_y * delta_y);

    // 设定一个极小值作为接近目标的阈值
    const float epsilon = 0.001f;

    if (*Distance_Expect < epsilon)
    {
        // 已接近目标点，无需移动和转向
        *Distance_Expect = 0.0f;
        *Yaw_Expect = 0.0f; // 到达目标点时偏航角设为0
    }
    else
    {
        // 计算目标偏航角（弧度），再转换为角度
        *Yaw_Expect = atan2f(delta_y, delta_x) * 180.0f / (float)M_PI;

        // 将角度转换到±180度范围
        if (*Yaw_Expect > 180.0f)
        {
            *Yaw_Expect -= 360.0f;
        }
        else if (*Yaw_Expect < -180.0f)
        {
            *Yaw_Expect += 360.0f;
        }
    }
}
extern float Distance_Measure;
extern float yaw_angle;
extern PIDTypeDef MotorPID[2];
float Yaw_Expect1, Distance_Expect1, X_Current1, Y_Current1;
float Yaw_Expect1, Distance_Expect1;
// 1. 关键：用static修饰，确保变量在函数多次调用间保持状态（不被重置）
static float X_Current1 = 0.0f;
static float Y_Current1 = 0.0f;
static int16_t Flag = 0;
static int16_t Flag_Cal = 0;
extern int32_t Encoder_Interral[2];
static uint8_t Motion_Stage = 0;
extern float yaw_Init;
#define YAW_ERROR_THRESHOLD 1.0f

void Xunhang(struct Places_Type places[8])
{
    // 1. 所有目标点完成：停止电机
    if (Flag >= 8)
    {
        MotorPID[0].RPMSetting = 0.0f;
        MotorPID[1].RPMSetting = 0.0f;
        return;
    }

    // 2. 计算目标航向和距离（仅首次进入当前目标点时计算1次）
    if (Flag_Cal == 0)
    {
        // 计算：从当前位置到第Flag个目标点的航向和距离
        Yaw_Distance_Calculate(&Yaw_Expect1, &Distance_Expect1,
                               places[Flag].x, places[Flag].y,
                               X_Current1, Y_Current1);

        Yaw.YawSetting = yaw_Init + Yaw_Expect1; // 直接赋值目标航向（非累加，避免偏差）
                                                 // 确保目标航向在±180°范围内（与原有逻辑一致）
        if (Yaw.YawSetting > 180.0f)
            Yaw.YawSetting -= 360.0f;
        else if (Yaw.YawSetting < -180.0f)
            Yaw.YawSetting += 360.0f;
        Flag_Cal = 1;     // 标记：已计算目标参数
        Motion_Stage = 0; // 初始阶段：转向
    }

    // 3. 阶段1：转向（仅调整航向，不移动）
    if (Motion_Stage == 0)
    {
        // 计算航向偏差（当前航向 - 目标航向）
        float yaw_error = yaw_angle - Yaw.YawSetting;
        // 修正偏差范围（确保在±180°内，避免绕远）
        if (yaw_error > 180.0f)
            yaw_error -= 360.0f;
        else if (yaw_error < -180.0f)
            yaw_error += 360.0f;

        // 仅输出转向PID（电机差速实现转向，无直行速度）
        float yaw_pid_out = Yaw.FunctionCalPID(&Yaw, yaw_angle);
        MotorPID[0].RPMSetting = yaw_pid_out;  // 左电机：+转向输出
        MotorPID[1].RPMSetting = -yaw_pid_out; // 右电机：-转向输出（差速转向）

        // 转向完成判断：航向偏差小于阈值（如±2°）
        if (fabs(yaw_error) < YAW_ERROR_THRESHOLD)
        {
            Motion_Stage = 1;        // 切换到：直行阶段
            Distance_Measure = 0.0f; // 重置距离测量（准备直行）
            Encoder_Interral[0] = 0; // 重置编码器积分（确保距离从0开始）
            Encoder_Interral[1] = 0;
        }
    }

    // 4. 阶段2：直行（锁定航向，仅修正微小偏差，直线移动）
    else if (Motion_Stage == 1)
    {
        // 4.1 直行PID：计算直行速度（控制直线移动）
        float distance_pid_out = Distance.FunctionCalPID(&Distance, Distance_Measure);
        // 4.2 航向修正PID：仅用小系数修正偏差（锁定航向，避免大转向）
        float yaw_error = yaw_angle - Yaw.YawSetting;
        if (yaw_error > 180.0f)
            yaw_error -= 360.0f;
        else if (yaw_error < -180.0f)
            yaw_error += 360.0f;
        // 新增：航向修正系数（0.1-0.3，越小越稳定，避免影响直线）
        float yaw_correction = Yaw.FunctionCalPID(&Yaw, yaw_angle) * 0.2f;

        // 电机输出：直行为主，小修正为辅（确保直线）
        MotorPID[0].RPMSetting = distance_pid_out + yaw_correction;
        MotorPID[1].RPMSetting = distance_pid_out - yaw_correction;

        // 4.3 到达目标点判断：距离偏差小于阈值（0.1f）
        if (fabs(Distance_Measure - Distance_Expect1) < 0.1f)
        {
            Flag++;                          // 切换到下一个目标点
            X_Current1 = places[Flag - 1].x; // 更新当前位置（刚到达的点）
            Y_Current1 = places[Flag - 1].y;
            LED_Blink(2);     // 到达提示
            Flag_Cal = 0;     // 重置计算标记（准备下一个点）
            Motion_Stage = 0; // 重置阶段：下一个点先转向
        }
    }
}
void LED_Flash(int times)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(times);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(times);
}
void LED_Blink(int times)
{
    while (times-- > 0)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(200);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(200);
    }
}
