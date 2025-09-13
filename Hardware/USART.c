#include "USART.h"
#include "usart.h"
#include <string.h> // 包含 memset 函数的声明
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
uint8_t g_recv_buf[11] = {0};
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
/* 判断是哪个串口触发的中断 */
// if (huart == &huart1)
//{
// HAL_UART_Receive_IT(&huart1, UART1RecvBuff, UART1_RECV_BUFF_LEN);
//}

//  if (huart == &huart6)
//  {
//    // 第二步：仅处理“接收未完成”的情况（bit15=0）
//    if ((UART6_RX_STA & 0x8000) == 0)
//    {
//      // 1. 存储当前接收的1字节到缓冲区（低14位记字节数，避免溢出）
//      UART6_RX_BUF[UART6_RX_STA & 0x3FFF] = Res_UART6;
//      UART6_RX_STA++; // 字节数+1

//      // 2. 双帧头校验（前2字节必须是0x5A，否则重置）
//      if (UART6_RX_STA == 1)
//      { // 第1字节：帧头1
//        if (UART6_RX_BUF[0] != 0x5A)
//        {
//          UART6_RX_STA = 0; // 帧头错，丢弃数据
//        }
//      }
//      else if (UART6_RX_STA == 2)
//      { // 第2字节：帧头2
//        if (UART6_RX_BUF[1] != 0x5A)
//        {
//          UART6_RX_STA = 0; // 帧头错，丢弃数据
//        }
//      }

//      // 3. 关键补充：接收够11字节 → 置位bit15（标记接收完成）
//      if (UART6_RX_STA >= 11)
//      {
//        UART6_RX_STA |= 0x8000; // 置位bit15，明确“接收完成”
//      }

//      // 4. 缓冲区溢出保护（超过最大长度，强制重置）
//      if (UART6_RX_STA > (sizeof(UART6_RX_BUF) - 1))
//      {
//        UART6_RX_STA = 0;
//      }
//    }

//    // 5. 当“接收完成”（bit15=1）时，执行校验和与数据解析
//    if ((UART6_RX_STA & 0x8000) == 1)
//    {
//      // （1）计算校验和：前10字节累加，保留低8位
//      CheckSum_UART6 = 0;
//      for (uint8_t i = 0; i < 10; i++)
//      {
//        CheckSum_UART6 += UART6_RX_BUF[i];
//        CheckSum_UART6 &= 0xFF; // 防止溢出（仅保留8位）
//      }

//      // （2）校验和对比：一致则解析数据
//      if (UART6_RX_BUF[10] == CheckSum_UART6)
//      {
//        // 解析3组2字节数据（Byte4~Byte9，高字节在前，可按需调整字节序）
//        ParsedData_UART6[0] = (UART6_RX_BUF[4] << 8) | UART6_RX_BUF[5];
//        ParsedData_UART6[1] = (UART6_RX_BUF[6] << 8) | UART6_RX_BUF[7];
//        ParsedData_UART6[2] = (UART6_RX_BUF[8] << 8) | UART6_RX_BUF[9];
//        printf("Roll:%.1d,Pitch:%.1d,Yaw:%.1d/r/n", ParsedData_UART6[0], ParsedData_UART6[1], ParsedData_UART6[2]);

//        // TODO：添加业务逻辑（如使用解析后的ParsedData_UART6）
//        // 示例：printf("UART6解析：D1=0x%04X, D2=0x%04X, D3=0x%04X\n",
//        //        ParsedData_UART6[0], ParsedData_UART6[1], ParsedData_UART6[2]);
//      }
//      else
//      {
//        // 校验和错误（可选：错误处理，如LED闪烁）
//        // printf("UART6校验错：实际0x%02X，计算0x%02X\n", UART6_RX_BUF[10], CheckSum_UART6);
//      }

//      // 6. 重置接收状态，准备下一帧
//      UART6_RX_STA = 0;    // 字节数归0 + bit15清0
//      UART6_RX_BUF[0] = 0; // 清空第1字节，避免帧头误判
//    }

//    // 第三步：重新开启UART6中断接收（HAL库必须重开，否则仅接收1次）
//    HAL_UART_Receive_IT(&huart6, &Res_UART6, 1);
//  }
//  HAL_UART_Receive_IT(&huart6, &Res_UART6, 1);
//}
void UART_BuffInit(UARTTypeDef *uart, uint8_t commNum)
{
    uint16_t i;
    uart->CommNum = commNum;
    uart->RecvBuffSize = UART_RECV_BUFF_SIZE;
    uart->SendBuffSize = UART_SEND_BUFF_SIZE;

    uart->RecvBuffAvailablePoints = 0;
    uart->RecvPopIndex = 0;
    uart->RecvPushIndex = 0;
    for (i = 0; i < uart->RecvBuffSize; i++)
    {
        uart->RecvBuff[i] = 0;
    }

    uart->SendBuffAvailablePoints = 0;
    uart->SendPopIndex = 0;
    uart->SendPushIndex = 0;
    for (i = 0; i < uart->SendBuffSize; i++)
    {
        uart->SendBuff[i] = 0;
    }
}

int UART_PushDataToSendFIFOBuffer(UARTTypeDef *uart, uint8_t *pdata, uint16_t len)
{
    uint16_t i;

    if (uart->SendBuffAvailablePoints == uart->SendBuffSize)
    {
        return 0;
    }

    if ((uart->SendBuffAvailablePoints + len) <= uart->SendBuffSize)
    {
        for (i = 0; i < len; i++)
        {
            uart->SendBuff[uart->SendPushIndex] = pdata[i];
            uart->SendPushIndex++;
            if (uart->SendPushIndex == uart->SendBuffSize)
            {
                uart->SendPushIndex = 0;
            }
            uart->SendBuffAvailablePoints++;
        }
        return 1;
    }
    return 0;
}

int UART_PopDataFromSendFIFOBuffer(UARTTypeDef *uart, uint8_t *pdata, uint16_t len)
{
    uint16_t i;

    if (uart->SendBuffAvailablePoints == 0)
    {
        return 0;
    }
    if (uart->SendBuffAvailablePoints >= len)
    {
        for (i = 0; i < len; i++)
        {
            pdata[i] = uart->SendBuff[uart->SendPopIndex];
            uart->SendPopIndex++;
            if (uart->SendPopIndex == uart->SendBuffSize)
            {
                uart->SendPopIndex = 0;
            }
            uart->SendBuffAvailablePoints--;
        }
        return len;
    }

    return 0;
}

int fputc(int ch, FILE *stream)
{
    // 阻塞发送1个字符到USART1（直到发送完成，避免数据丢失）
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch; // 返回发送的字符，确保printf正常工作
}

/*Vofa数据解析赋值部分*/
uint8_t id_Flag;         // 1为Kp 2为Ki 3为Kd
uint8_t Data_BitNum = 4; // 数据的位数，即12.123 有6位 -12.123有7为
// 串口中断，用于接受vofa的参数的   #P1=12.123！   #P为帧头，1为是改变谁的标志位， =是数据收集标志位
float PID_Pitch[3] = {0.0, 0.0, 0.0}; // Kp,Ki,Kd
uint8_t PID_index = 0;
int16_t USART1_ReceiveData;
extern Pitch_PIDTypeDef Pit;
uint8_t Usart_RxData;
uint8_t Usart_RxFlag;
uint8_t Usart_RxPacket[100];      // 接受数据包
uint8_t Usart_RxPacket_Len = 100; // 接受数据包长度
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    /* 判断是哪个串口触发的中断 */
    if (huart == &huart1)
    {

        {
            HAL_UART_Receive_IT(&huart1, &Usart_RxData, 1); // 重新开启中断
            if (RxState == 0 && Usart_RxData == 0x23)       // 第一个帧头  "#"==0x23
            {
                RxState = 1;
            }
            else if (RxState == 1 && Usart_RxData == 0x50) // 第二个帧头  "P"==0x50
            {
                RxState = 2;
            }
            else if (RxState == 2) // 确认传参的对象 即修改id_Flag
            {
                id_Flag = Usart_RxData - 48;
                RxState = 3;
            }
            else if (RxState == 3 && Usart_RxData == 0x3D) // 判断等号，也可以类比为数据开始的帧头
            {
                RxState = 4;
            }
            else if (RxState == 4) // 开始接收传输的数据
            {
                if (Usart_RxData == 0x21) // 结束的帧尾   如果没有接收到！即还有数据来，就一直接收
                {
                    Data_BitNum = pRxPacket; // 获取位数
                    pRxPacket = 0;           // 清除索引方便下次进行接收数据
                    RxState = 0;
                    Usart_RxFlag = 1;
                    PID_index = Get_id_Flag();
                    PID_Pitch[PID_index - 1] = RxPacket_Data_Handle();
                    Pit.KP = PID_Pitch[0];
                    Pit.KI = PID_Pitch[1];
                    Pit.KD = PID_Pitch[2];
                }
                else
                {
                    Usart_RxPacket[pRxPacket++] = Usart_RxData; // 把数据放在数据包内
                }
            }
        }
    }
    HAL_UART_Receive_IT(&huart1, &Usart_RxData, 1); // 重新开启中断
}

uint8_t Get_id_Flag(void) // 将获取id_Flag封装成函数
{
    uint8_t id_temp;
    id_temp = id_Flag;
    id_Flag = 0;
    return id_temp;
}

float Pow_invert(uint8_t X, uint8_t n) // x除以n次10
{
    float result = X;
    while (n--)
    {
        result /= 10;
    }
    return result;
}

// uint8_t Usart_RxPacket[5]={0x31,0x32,0x2E,0x31,0x33};//可以给数据包直接赋值直接调用一下换算程序，看是否输出为12.13
// Data_BitNum = 5//别忘记数据的长度也要设置
// 然后直接在主程序就放  Usart_Printf("%f\n",RxPacket_Data_Handle());  Delay_ms(1000);就ok了
float RxPacket_Data_Handle(void) // 数据包换算处理
{
    float Data = 0.0;
    uint8_t dot_Flag = 0;      // 小数点标志位，能区分小数点后或小数点前 0为小数点前，1为小数点后
    uint8_t dot_after_num = 1; // 小数点后的第几位
    int8_t minus_Flag = 1;     // 负号标志位 -1为是负号 1为正号
    for (uint8_t i = 0; i < Data_BitNum; i++)
    {
        if (Usart_RxPacket[i] == 0x2D) // 如果第一位为负号
        {
            minus_Flag = -1;
            continue; // 跳过本次循环
        }
        if (dot_Flag == 0)
        {
            if (Usart_RxPacket[i] == 0x2E) // 如果识别到小数点，则将dot_Flag置1
            {
                dot_Flag = 1;
            }
            else // 还没遇到小数点前的运算
            {
                Data = Data * 10 + Usart_RxPacket[i] - 48;
            }
        }
        else // 遇到小数点后的运算
        {
            Data = Data + Pow_invert(Usart_RxPacket[i] - 48, dot_after_num);
            dot_after_num++;
        }
    }
    return Data * minus_Flag; // 将换算后的数据返回出来 这里乘上负号标志位
}
