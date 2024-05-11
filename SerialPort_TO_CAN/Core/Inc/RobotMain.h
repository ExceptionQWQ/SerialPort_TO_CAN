/*
 * @author BusyBox
 * @date 2024/5/11
 * @version 1.0
 * @git https://github.com/ExceptionQWQ/SerialPort_TO_CAN
 */

#pragma once

#include "can.h"
#include "usart.h"
#include "dma.h"
#include "gpio.h"

#ifdef __cplusplus

#include "rdk/core/transfer/crc.h"
#include "rdk/core/transfer/circular_buffer.h"
#include "rdk/core/transfer/serial_port.h"
#include "rdk/core/transfer/serial_to_can.h"

#endif


#ifdef __cplusplus
extern "C" {
#endif

    void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
    void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart);

    void RobotInit();
    void RobotMain();

#ifdef __cplusplus
};
#endif