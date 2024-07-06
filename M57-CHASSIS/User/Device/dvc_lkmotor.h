/**
 * @file dvc_dmmotor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 겿ص�����������
 * @version 0.1
 * @date 2023-08-30 0.1 ����
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_LKMOTOR_H
#define DVC_LKMOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "alg_pid.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ������״̬
 *
 */
enum Enum_LK_Motor_Status
{
    LK_Motor_Status_DISABLE = 0,
    LK_Motor_Status_ENABLE,
};

/**
 * @brief ��������IDö������
 *
 */
enum Enum_LK_Motor_ID : uint8_t
{
    LK_Motor_ID_UNDEFINED = 0,
    LK_Motor_ID_0x141,
    LK_Motor_ID_0x142,
    LK_Motor_ID_0x143,
    LK_Motor_ID_0x144,
    LK_Motor_ID_0x145,
    LK_Motor_ID_0x146,
    LK_Motor_ID_0x147,
    LK_Motor_ID_0x148,
};

/**
 * @brief ����������״̬
 *
 */
enum Enum_LK_Motor_Control_Status
{
    LK_Motor_Control_Status_DISABLE = 0,
    LK_Motor_Control_Status_ENABLE,
};

/**
 * @brief ������can����cmd_id
 *
 */
enum Enum_LK_Motor_Control_ID : uint8_t
{
    LK_Motor_Control_Shut_Down = 0x80,   //����ر�?
    LK_Motor_Control_Stop = 0x81, //���ͣ�?
    LK_Motor_Control_Run = 0x88,//�������?
    LK_Motor_Control_Torque = 0xA1,//���رջ�����
};

/**
 * @brief ���������Ʒ�ʽ
 *
 */
enum Enum_LK_Motor_Control_Method
{
    LK_Motor_Control_Method_IMU_ANGLE = 0,
    LK_Motor_Control_Method_IMU_OMEGA,
    LK_Motor_Control_Method_ANGLE,
    LK_Motor_Control_Method_OMEGA,
    LK_Motor_Control_Method_TORQUE,
};

/**
 * @brief ������Դ����
 *
 */
struct Struct_LK_Motor_CAN_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID;
    uint8_t Temperature_Centigrade;  //���϶�
    uint16_t Current_Reverse;
    uint16_t Omega_Reverse;
    uint16_t Encoder_Reverse;
} __attribute__((packed));

/**
 * @brief ��������������������, Ť�طǹ��ʵ�λ��
 *
 */
struct Struct_LK_Motor_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID;  //��������ID
    float Now_Angle;  //�Ƕ���
    float Now_Omega;  //������
    float Now_Current;  //����
    float Now_Temperature; //���϶�
    uint16_t Pre_Encoder; 
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief LK��ˢ���?, ��Ƭ��������������?
 * DM_Motor_Control_Method_POSITION_OMEGAģʽ��, ��������ָ�������λ�û�PI����, ����250��0
 * 
 * PMAXֵ���ڵ�����������Ϊ3.141593, ��PI, ��ʱ����IMUģʽ�µ����ʹ��?
 *
 */
class Class_LK_Motor
{
public:
    // PID�ǶȻ�����
    Class_PID PID_Angle;
    // PID���ٶȻ�����
    Class_PID PID_Omega;
    // PIDŤ�ػ�����
    Class_PID PID_Torque;

    void Init(CAN_HandleTypeDef *hcan, Enum_LK_Motor_ID __CAN_ID, float __Omega_Max, int32_t __Position_Offset = 0, float __Current_Max = 33.0f ,Enum_LK_Motor_Control_Method __Control_Method = LK_Motor_Control_Method_IMU_ANGLE);

    inline Enum_LK_Motor_Control_Status Get_LK_Motor_Control_Status();
    inline Enum_LK_Motor_Status Get_LK_Motor_Status();
    inline float Get_Output_Max();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_Temperature();
    inline Enum_LK_Motor_Control_Method Get_Control_Method();
    inline float Get_IMU_K_P();
    inline float Get_IMU_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    
    inline void Set_LK_Control_Status(Enum_LK_Motor_Control_Status __DM_Motor_Control_Status);
    inline void Set_LK_Motor_Control_Method(Enum_LK_Motor_Control_Method __DM_Motor_Control_Method);
    inline void Set_IMU_K_P(float __IMU_K_P);
    inline void Set_IMU_K_D(float __IMU_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Current(float __Target_Current);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    //��ʼ����ر���?

    //�󶨵�CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //�����ݰ󶨵�CAN ID, ����֡��0xxa1~0xxaf
    Enum_LK_Motor_ID CAN_ID;
    //���ͻ�����
    uint8_t *CAN_Tx_Data;
    //λ�÷���ƫ��
    uint32_t Position_Offset;
    //����ٶ�?, ������������, �Ƽ�20.94359, Ҳ��������?��200rpm
    float Omega_Max;
    //���Ť��?, ������������, �Ƽ�7, Ҳ����������7NM
    float Current_Max;
    //����͵����?
    const int16_t Current_Max_Cmd = 2000;
    //���������?
    float Out = 0.0f;
    //����
    
    const float Torque_Current = 0.3;  //Ϲд�� ����ת��ϵ��

    //һȦλ�ÿ̶�
    uint32_t Position_Max = 16383;


    
    //�ڲ�����

    //��ǰʱ�̵ĵ������flag
    uint32_t Flag = 0;
    //ǰһʱ�̵ĵ������flag
    uint32_t Pre_Flag = 0;

    //������

    //���״�?
    Enum_LK_Motor_Status LK_Motor_Status = LK_Motor_Status_DISABLE;
    //�������ӿ���Ϣ
    Struct_LK_Motor_Rx_Data Data;

    //д����

    //��д����

    //�����������ģ�?
    Enum_LK_Motor_Control_ID LK_Motor_Control_ID = LK_Motor_Control_Torque;
    //�������״�?
    Enum_LK_Motor_Control_Status LK_Motor_Control_Status = LK_Motor_Control_Status_DISABLE;
    //������Ʒ��?
    Enum_LK_Motor_Control_Method LK_Motor_Control_Method = LK_Motor_Control_Method_IMU_ANGLE;
    //MIT��Kpֵ, 0~500, ����6, λ�ÿ�����Ҫ
    float IMU_K_P = 0.0f;
    //MIT��Kdֵ, 0~5, ����0.2, λ�ú��ٶȿ�����Ҫ
    float IMU_K_D = 0.0f;
    //Ŀ��ĽǶ�?, rad
    float Target_Angle = 0.0f;
    //Ŀ����ٶ�?, rad/s
    float Target_Omega = 0.0f;
    //Ŀ��ĵ���?
    float Target_Current = 0.0f;
    //Ŀ������
    float Target_Torque = 0.0f;
    //�ڲ�����
 
    void Output(void);
    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief ��ȡ���״�?
 *
 * @return Enum_LK_Motor_Status ���״�?
 */
Enum_LK_Motor_Status Class_LK_Motor::Get_LK_Motor_Status()
{
    return (LK_Motor_Status);
}


/**
 * @brief ��ȡ���״�?
 *
 * @return Enum_LK_Motor_Status ���״�?
 */
Enum_LK_Motor_Control_Status Class_LK_Motor::Get_LK_Motor_Control_Status()
{
    return (LK_Motor_Control_Status);
}


/**
 * @brief ��ȡ��ǰ�ĽǶ�, rad
 *
 * @return float ��ǰ�ĽǶ�, rad
 */
float Class_LK_Motor::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief ��ȡ��ǰ���ٶ�, rad/s
 *
 * @return float ��ǰ���ٶ�, rad/s
 */
float Class_LK_Motor::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief ��ȡ��ǰ��Ť��, ֱ�Ӳ��÷���ֵ
 *
 * @return float ��ǰ��Ť��, ֱ�Ӳ��÷���ֵ
 */
float Class_LK_Motor::Get_Now_Torque()
{
    return (Data.Now_Current);
}

/**
 * @brief ��ȡ��ǰMOS�ܵ��¶�, ���϶�
 *
 * @return float ��ǰMOS�ܵ��¶�, ���϶�
 */
float Class_LK_Motor::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief ��ȡ������Ʒ��?
 *
 * @return Enum_LK_Motor_Control_Method ������Ʒ��?
 */
Enum_LK_Motor_Control_Method Class_LK_Motor::Get_Control_Method()
{
    return (LK_Motor_Control_Method);
}

/**
 * @brief ��ȡMIT��Kpֵ, 0~500
 *
 * @return float MIT��Kpֵ, 0~500
 */
float Class_LK_Motor::Get_IMU_K_P()
{
    return (IMU_K_P);
}

/**
 * @brief ��ȡMIT��Kdֵ, 0~5
 *
 * @return float MIT��Kdֵ, 0~5
 */
float Class_LK_Motor::Get_IMU_K_D()
{
    return (IMU_K_D);
}

/**
 * @brief ��ȡĿ��ĽǶ�?, rad
 *
 * @return float Ŀ��ĽǶ�?, rad
 */
float Class_LK_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief ��ȡĿ����ٶ�?, rad/s
 *
 * @return float Ŀ����ٶ�?, rad/s
 */
float Class_LK_Motor::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief ��ȡĿ���Ť��?
 *
 * @return float Ŀ���Ť��?
 */
float Class_LK_Motor::Get_Target_Torque()
{
    return (Target_Current);
}

/**
 * @brief �趨�������״�?
 *
 * @param __DM_Motor_Control_Status �������״�?
 */
void Class_LK_Motor::Set_LK_Control_Status(Enum_LK_Motor_Control_Status __DM_Motor_Control_Status)
{
    LK_Motor_Control_Status = __DM_Motor_Control_Status;
}

/**
 * @brief �趨������Ʒ��?
 *
 * @param __Control_Method ������Ʒ��?
 */
void Class_LK_Motor::Set_LK_Motor_Control_Method(Enum_LK_Motor_Control_Method __Control_Method)
{
    LK_Motor_Control_Method = __Control_Method;
}

/**
 * @brief �趨MIT��Kpֵ, 0~500, ����6, λ�ÿ�����Ҫ
 *
 * @param __MIT_K_P MIT��Kpֵ, 0~500, ����6, λ�ÿ�����Ҫ
 */
void Class_LK_Motor::Set_IMU_K_P(float __IMU_K_P)
{
    IMU_K_P = __IMU_K_P;
}

/**
 * @brief �趨MIT��Kdֵ, 0~5, ����0.2, λ�ú��ٶȿ�����Ҫ
 *
 * @param __MIT_K_D MIT��Kdֵ, 0~5, ����0.2, λ�ú��ٶȿ�����Ҫ
 */
void Class_LK_Motor::Set_IMU_K_D(float __IMU_K_D)
{
    IMU_K_D = __IMU_K_D;
}

/**
 * @brief �趨Ŀ��ĽǶ�?, rad
 *
 * @param __Target_Angle Ŀ��ĽǶ�?, rad
 */
void Class_LK_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief �趨Ŀ����ٶ�?, rad/s
 *
 * @param __Target_Omega Ŀ����ٶ�?, rad/s
 */
void Class_LK_Motor::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief �趨Ŀ������ص���?
 *
 * @param __Target_Current Ŀ������ص���?
 */
void Class_LK_Motor::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

/**
 * @brief �趨Ŀ������ص���?
 *
 * @param __Target_Torque Ŀ������ص���?
 */
void Class_LK_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

void Class_LK_Motor::Set_Out(float __Out)
{
    if(__Out > Current_Max_Cmd)
    {
        __Out = Current_Max_Cmd;
    }
    else if(__Out < -Current_Max_Cmd)
    {
        __Out = -Current_Max_Cmd;
    }
}

float Class_LK_Motor::Get_Output_Max()
{
    return (Current_Max_Cmd);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
