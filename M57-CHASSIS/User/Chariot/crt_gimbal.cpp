/**
 * @file crt_gimbal.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

float __Target_Omega=3;
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环速度控制
        Set_Out(Target_Omega / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Yaw);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
        }
        else
        {
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(True_Gyro_Yaw);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::Transform_Angle()
{
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw();
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环速度控制
        Set_Out(Target_Omega / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);
        }
        else
        {

            PID_Angle.Set_Now(RAD_TO_ANGEL(True_Rad_Pitch));
            PID_Omega.Set_Now(RADPS_TO_RPM(True_Gyro_Pitch));
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Target_Torque = -PID_Omega.Get_Out();
        Set_Out(Target_Torque);
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = -PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
        }
        else
        {
            PID_Angle.Set_Now(RAD_TO_ANGEL(True_Rad_Pitch));
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(RADPS_TO_RPM(True_Gyro_Pitch));
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = -PID_Omega.Get_Out();
        Set_Out(Target_Torque + Gravity_Compensate);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::Transform_Angle()
{
    True_Rad_Pitch = 1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = -1 * IMU->Get_Gyro_Pitch();
    True_Angle_Pitch = RADPS_TO_RPM(True_Rad_Pitch);
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::TIM_PID_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
    case (LK_Motor_Control_Method_TORQUE):
    {
        Out = Target_Torque * Torque_Current / Current_Max * Current_Max_Cmd;
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();
            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
        }
        else
        {
            PID_Angle.Set_Now(True_Rad_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();
            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(True_Gyro_Pitch);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = (PID_Omega.Get_Out() + Gravity_Compensate);
        Set_Out(Out);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::Transform_Angle()
{
    True_Rad_Pitch = -1 * IMU->Get_Rad_Roll();
    True_Gyro_Pitch = -1 * IMU->Get_Gyro_Roll();
    True_Angle_Pitch = -1 * IMU->Get_Angle_Roll();
}

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    
    // PID参数列表：(P, I, D, F, I_Out_Max, Out_Max, I_Variable_Speed_A, I_Variable_Speed_B, I_Separate_Threshold, T, Dead_Zone, D_First)

    // yaw轴电机
    Motor_Yaw.PID_Angle.Init(0.175f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0, 0.001, 0.0);
    Motor_Yaw.PID_Omega.Init(25000.0f, 2000.0f, 1.0f, 0.0f, 8000, 24500,0,0,0,0.001,0);

    Motor_Yaw.PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE, 2048);
    // pitch轴电机 LK6010
    Motor_Pitch_LK6010.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 6.0f * PI, 6.0f * PI);
    Motor_Pitch_LK6010.PID_Omega.Init(150.0f, 150.0f, 0.0f, 0, 500, 900);
    Motor_Pitch_LK6010.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch_LK6010.Get_Output_Max(), Motor_Pitch_LK6010.Get_Output_Max());
    Motor_Pitch_LK6010.IMU = &Boardc_BMI;
    Motor_Pitch_LK6010.Init(&hcan1, LK_Motor_ID_0x141, DJI_Motor_Control_Method_IMU_ANGLE);
}

/**
 * @brief 输出到电机
 *
 */
static float test_speed1 = 1.8f;
static float test_speed2 = -CRUISE_PITCH;
static uint16_t delay_time = 0;
float f_d;
extern uint8_t Data_Sign;

void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch_LK6010.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_LK6010.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_LK6010.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Omega(0.0f);
        Motor_Pitch.Set_Target_Omega(0.0f);
        Motor_Pitch_LK6010.Set_Target_Omega(0.0f);

        Motor_Yaw.Set_Target_Angle(Motor_Yaw.Get_True_Angle_Yaw());
        Motor_Pitch_LK6010.Set_Target_Angle(Motor_Pitch_LK6010.Get_True_Angle_Pitch());
    }
    else
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            // 云台工作
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
            Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);

            // 限制角度范围 处理yaw轴180度问题
            if ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
            {
                Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
            }
            else if ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
            {
                Target_Yaw_Angle += (2 * Max_Yaw_Angle);
            }

            // pitch限位
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Rad, Max_Pitch_Rad);
            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            // 云台巡航状态
            if(MiniPC->Get_Gimbal_Control_Mode()==0)
            {
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_OMEGA);
                if(MiniPC->Get_Chassis_Target_Velocity_X() == 0 && MiniPC->Get_Chassis_Target_Velocity_Y() == 0)
                {
                    Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_OMEGA);
                    Motor_Yaw.Set_Target_Omega(test_speed1);        // 设定yaw轴速度
                    
                    if(Motor_Pitch_LK6010.Get_True_Angle_Pitch()>=0)// 计算pitch轴速度
                        test_speed2 = -CRUISE_PITCH;
                    else if(Motor_Pitch_LK6010.Get_True_Angle_Pitch()<=-18.5)
                        test_speed2 =  CRUISE_PITCH;
                    
                    if(delay_time <= 1000)                          // 导航巡航 -> 巡航延迟消抖
                    {
                        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
                        Motor_Pitch_LK6010.Set_Target_Angle(-0.146f);
                        delay_time++;
                    }
                    else
                    {
                        Motor_Pitch_LK6010.Set_Target_Omega(test_speed2);
                        Target_Pitch_Angle = Motor_Pitch_LK6010.Get_True_Angle_Pitch()*PI/180;  // 获取pitch轴角度
                        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Rad, Max_Pitch_Rad);      // pitch限位
                        Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);                // 设置pitch轴角度
                    }
                }
                else
                {
                    Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE); 

                    Motor_Yaw.Set_Target_Omega(test_speed1*0.5);
                    Motor_Pitch_LK6010.Set_Target_Angle(-0.146f);
                    delay_time = 0;
                }

                Target_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw();                      // 设置yaw轴角度
                Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            }

            else if(MiniPC->Get_Gimbal_Control_Mode()==1)   // 云台非巡航状态
            {
                // 云台工作
                if(Data_Sign == 0)                          // 自瞄
                {
                    Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
                    Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
                    Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);

                    Motor_Yaw.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle());             // 设置yaw角度

                    if ((Motor_Yaw.Get_Target_Angle() - Motor_Yaw.Get_True_Angle_Yaw()) > 180)
                    {
                        Motor_Yaw.Set_Target_Angle(Motor_Yaw.Get_Target_Angle() - 360);
                    }
                    if ((Motor_Yaw.Get_Target_Angle() - Motor_Yaw.Get_True_Angle_Yaw()) < -180)
                    {
                        Motor_Yaw.Set_Target_Angle(Motor_Yaw.Get_Target_Angle() + 360);
                    }

                    Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();                  // 获取pitch轴角度
                    Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Rad, Max_Pitch_Rad);  // pitch限位
                    Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);            // 设置pitch轴角度
                }
                else if(Data_Sign == 1)                     // 导航
                {
                    // // 云台工作
                    // Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);

                    // Target_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw();                      // 设置yaw轴角度
                    // Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);

                    // 云台工作
                    Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_OMEGA);
                    Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);

                    Target_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw();                      // 设置yaw轴角度    
                    Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
                    float Target_Yaw_Omega = MiniPC->Get_Gimbal_Target_Omega();             // 设置yaw角速度
                    Motor_Yaw.Set_Target_Omega(Target_Yaw_Omega);
                    

                    Target_Pitch_Angle = 0.0f;  // 设置pitch为0°
                    Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Rad, Max_Pitch_Rad);      // pitch限位
                    Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);                // 设置pitch轴角度 
                }
            }
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {
            if(MiniPC->Get_Gimbal_Control_Mode()==0)
            {
                // 上位机在巡航中途离线
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_OMEGA);
                Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_OMEGA);
                Motor_Yaw.Set_Target_Omega(test_speed1);            // 设定yaw轴速度

                if(Motor_Pitch_LK6010.Get_True_Angle_Pitch()>=0)    // 计算pitch轴速度
                    test_speed2 = -CRUISE_PITCH;
                else if(Motor_Pitch_LK6010.Get_True_Angle_Pitch()<=-8)
                    test_speed2 =  CRUISE_PITCH;

                if(delay_time <= 1000)                              // 导航巡航 -> 巡航延迟消抖
                {
                    Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
                    Motor_Pitch_LK6010.Set_Target_Angle(-0.146f);
                    delay_time++;
                }
                else
                    Motor_Pitch_LK6010.Set_Target_Omega(test_speed2);
            }
            else
            {
                // 上位机非巡航中途离线
                // 云台工作
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
                Motor_Pitch_LK6010.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);

                // 设置Yaw角度
                Target_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw();
                Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);

                // 设置Pitch角度
                Target_Pitch_Angle = Motor_Pitch_LK6010.Get_True_Angle_Pitch()*PI/180;
                Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Rad, Max_Pitch_Rad);      // pitch限位
                Motor_Pitch_LK6010.Set_Target_Angle(Target_Pitch_Angle);
            }
        }
    }
}

/**
 * @brief 计算云台yaw总角度
 *
 */
void Class_Gimbal::Calculate_Total_Angle()
{
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    // 根据不同c板的放置方式来修改这几个函数
    Motor_Yaw.Transform_Angle();
    Motor_Pitch.Transform_Angle();
    Motor_Pitch_LK6010.Transform_Angle();

    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_LK6010.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
