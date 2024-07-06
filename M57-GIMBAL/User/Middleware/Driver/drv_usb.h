/**
 * @file drv_uart.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief ����SCUT-Robotlab��д��UARTͨ�ų�ʼ������������
 * @version 0.1
 * @date 2023-08-29 0.1 23��������
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_USB_H
#define DRV_USB_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// �������ֽڳ���
#define USB_BUFFER_SIZE 48

/* Exported types ------------------------------------------------------------*/


/**
 * @brief USBͨ�Ž��ջص�������������
 *
 */
typedef void (*USB_Call_Back)(uint8_t *Buffer, uint32_t Length);


/**
 * @brief USBͨ�Ŵ����ṹ��
 *
 */
struct Struct_USB_Manage_Object
{
    uint8_t Tx_Buffer[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer[USB_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
	uint16_t Tx_Buffer_Length;
	USB_Call_Back Callback_Function;
};

/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/

void USB_Init(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object, USB_Call_Back __Callback_Function);
void TIM_USB_PeriodElapsedCallback(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
