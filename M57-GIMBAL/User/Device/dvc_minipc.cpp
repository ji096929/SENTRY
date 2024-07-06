/**
 * @file dvc_minipc.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 迷你主机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright ustc-robowalker (c) 2023
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"
#include "crt_gimbal.h"
#include "drv_math.h"

/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化
 *
 * @param __frame_header 数据包头标
 * @param __frame_rear 数据包尾标
 */
void Class_MiniPC::Init(Struct_USB_Manage_Object *__USB_Manage_Object, uint8_t __frame_header, uint8_t __frame_rear)
{
    USB_Manage_Object = __USB_Manage_Object;
    Frame_Header = __frame_header;
    Frame_Rear = __frame_rear;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{    
	if(!Verify_CRC16_Check_Sum(USB_Manage_Object->Rx_Buffer,USB_Manage_Object->Rx_Buffer_Length)) return;

    memcpy(&Data_NUC_To_MCU, USB_Manage_Object->Rx_Buffer, sizeof(Struct_MiniPC_Rx_Data));
    Self_aim(Data_NUC_To_MCU.Gimbal_Target_X, Data_NUC_To_MCU.Gimbal_Target_Y, Data_NUC_To_MCU.Gimbal_Target_Z, &Rx_Angle_Yaw, &Rx_Angle_Pitch, &Distance, &Error);
}

/**
 * @brief 迷你主机发送数据输出到usb发送缓冲区
 *
 */

extern can_rx1_t can_rx1;
extern can_rx2_t can_rx2;

void Class_MiniPC::Output()
{
    Data_MCU_To_NUC.header                         = Frame_Header;
    Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle         = Tx_Angle_Pitch;
    Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle           = -Tx_Angle_Yaw;
    Data_MCU_To_NUC.Gimbal_Now_Roll_Angle          = Tx_Angle_Roll;
    Data_MCU_To_NUC.Game_process                   = can_rx1.game_process;
    Data_MCU_To_NUC.Self_blood                     = can_rx1.self_blood;
    Data_MCU_To_NUC.Self_Outpost_HP                = can_rx1.self_outpost_HP;
    Data_MCU_To_NUC.Remaining_Time                 = can_rx1.time;
    Data_MCU_To_NUC.Oppo_Outpost_HP                = can_rx2.oppo_outpost_HP;
    Data_MCU_To_NUC.Self_Base_HP                   = can_rx2.self_base_HP;
    Data_MCU_To_NUC.Outpost_rfid                   = can_rx2.outpost_rfid;
    Data_MCU_To_NUC.Self_Color                     = can_rx2.color;
    Data_MCU_To_NUC.Projectile_allowance           = can_rx2.projectile_allowance_17mm;
    Data_MCU_To_NUC.Invincible_State               = can_rx2.invincible_state;
    Data_MCU_To_NUC.crc16                          = 0xffff;

    memcpy(USB_Manage_Object->Tx_Buffer, &Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));
    USB_Manage_Object->Tx_Buffer_Length = sizeof(Struct_MiniPC_Tx_Data);
    //crc16 校验
    Append_CRC16_Check_Sum(USB_Manage_Object->Tx_Buffer, sizeof(Struct_MiniPC_Tx_Data));
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
    Transform_Angle_Tx();
    Output();
}

/**
 * @brief usb通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *rx_data)
{
    // 滑动窗口, 判断迷你主机是否在线
    Flag += 1;
    Data_Process();
}

/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过迷你主机数据
    if (Flag == Pre_Flag)
    {
        // 迷你主机断开连接
        MiniPC_Status = MiniPC_Status_DISABLE;
    }
    else
    {
        // 迷你主机保持连接
        MiniPC_Status = MiniPC_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t ch_data;

    if (pchMessage == NULL)
        return 0xFFFF;
    while (dwLength--)
    {
        ch_data = *pchMessage++;
        wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
    }

    return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t w_expected = 0;

    if ((pchMessage == NULL) || (dwLength <= 2))
        return false;

    w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
    return (
        (w_expected & 0xff) == pchMessage[dwLength - 2] &&
        ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t w_crc = 0;

    if ((pchMessage == NULL) || (dwLength <= 2))
        return;

    w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

    pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

/**
 * 计算给定向量的偏航角（yaw）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float Class_MiniPC::calc_yaw(float x, float y, float z)
{
    // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
    if (x == 0)
    {
        return Gimbal_Yaw_Motor_GM6020->Get_True_Angle_Yaw();
    }

    float yaw = atan2f(y, x);

    // 将弧度制的偏航角转换为角度制
    yaw = (yaw * 180 / 3.1415926); // 向左为正，向右为负
    // yaw -= 2;
    return yaw;
}

/**
 * 计算给定向量的欧几里德距离。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float Class_MiniPC::calc_distance(float x, float y, float z)
{
    // 计算各分量的平方和，并取其平方根得到欧几里德距离
    float distance = sqrtf(x * x + y * y + z * z);

    return distance;
}

/**
 * 计算给定向量的俯仰角（pitch）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */

// uint8_t temp = 0;
float dz;
extern can_rx3_t can_rx3;
float Class_MiniPC::calc_pitch(float x, float y, float z)
{
    // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
    if (x == 0 || z == 0)
    {
        return IMU->Get_Angle_Pitch();
    }
    z=-z;
    float temp_hight;
    temp_hight = z;
    float pitch_t,pitch;
	pitch = atan2f(z, sqrtf(x*x+y*y));

    if(can_rx3.initial_speed < 20 || can_rx3.initial_speed>30)  can_rx3.initial_speed = 24;
   // 使用重力加速度模型迭代更新俯仰角
   for (uint8_t i = 0; i < 20; i++)
   {
       pitch = atanf(temp_hight/sqrtf(x*x+y*y));
       float v_x = (can_rx3.initial_speed) * cosf(pitch);
       float v_y = (can_rx3.initial_speed) * sinf(pitch);
       float k = 0.07;
    //    float t = sqrtf(x*x + y*y) / v_x;
       float t = (expf(k*sqrtf(x*x+y*y))-1)/(k*v_x);
       float h = v_y*t - 0.5*g*t*t;
       float dz = z - h;
       
       temp_hight += 0.5 * dz;

       if (fabsf(dz) < 0.001)
       {
           break;
       }

    //    根据 dz 和向量的欧几里德距离计算新的俯仰角的变化量，进行迭代更新
        //  pitch += asinf(dz / calc_distance(x, y, z));
   }
    return -pitch;
}
/**
 * 计算yaw，pitch
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */

uint8_t Data_Sign = 0;
void Class_MiniPC::Self_aim(float x, float y, float z, float *yaw, float *pitch, float *distance, float *error)
{
    if(x == 0 && y == 0 && z == 0)
        Data_Sign = 1;
    else
        Data_Sign = 0;
    *yaw = -calc_yaw(x, y, z);
    *pitch = -calc_pitch(x, y, z);
    *distance = calc_distance(x, y, z);
    *error = Calc_Error(x,y,z,Tx_Angle_Yaw,Tx_Angle_Pitch); //这里的z为上位机直接发的z，不是弹道解算后的z1，判断时存在一定误差（瞄准误差允许范围为5cm时，不影响10m内打弹）
}

/**
 * 计算当前瞄准点与目标点的偏差 (映射到同一球面的弦长)
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @param now_yaw   实际yaw轴角度（弧度制）
 * @param now_pitch 实际pitch轴角度（弧度制）
 * @return 偏差值
 */
float Class_MiniPC::Calc_Error(float x, float y, float z, float now_yaw, float now_pitch)
{
    float dis = sqrtf(x*x + y*y + z*z);
    float x0 = dis*cos(now_pitch)*cos(now_yaw);
    float y0 = dis*cos(now_pitch)*sin(now_yaw);
    float z0 = dis*sin(now_pitch);
    float err = sqrtf(pow(x-x0,2) + pow(y-y0,2) + pow(z-z0,2));
    return err;
}

float Class_MiniPC::meanFilter(float input)
{
    static float buffer[5] = {0};
    static uint64_t index = 0;
    float sum = 0;

    // Replace the oldest value with the new input value
    buffer[index] = input;

    // Increment the index, wrapping around to the start of the array if necessary
    index = (index + 1) % 5;

    // Calculate the sum of the buffer's values
    for (int i = 0; i < 5; i++)
    {
        sum += buffer[i];
    }

    // Return the mean of the buffer's values
    return sum / 5.0;
}

/************************ copyright(c) ustc-robowalker **************************/
