/*
 * @author BusyBox
 * @date 2024/5/11
 * @version 1.0
 * @git https://github.com/ExceptionQWQ/SerialPort_TO_CAN
 */

#include "RobotMain.h"

Crc8Calculator crc8_calculator(0x31);
CircularBuffer circular_buffer;
SerialPort serial_port(&huart1, &circular_buffer);
SimpleBinaryTransfer simple_binary_transfer(&serial_port, &crc8_calculator);
SerialToCan serial_to_can(&simple_binary_transfer, SerialToCan::IDType::EXT);

SerialToCan::IDType id_type = SerialToCan::IDType::STD;


void SetCanFifo0Filter()
{
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0; //筛选器组编号
    filter.FilterMode = CAN_FILTERMODE_IDMASK; //ID掩码模式
    filter.FilterScale = CAN_FILTERSCALE_32BIT; //32位长度
    //接收所有帧
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan, &filter);
}

void SetCanFifo1Filter()
{
    CAN_FilterTypeDef filter;
    filter.FilterBank = 1; //筛选器组编号
    filter.FilterMode = CAN_FILTERMODE_IDMASK; //ID掩码模式
    filter.FilterScale = CAN_FILTERSCALE_32BIT; //32位长度
    //接收所有帧
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;

    filter.FilterFIFOAssignment = CAN_RX_FIFO1;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan, &filter);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (id_type == SerialToCan::IDType::STD) { //标准帧

        CAN_RxHeaderTypeDef rx;
        SerialToCan::FrameData frame_data;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, reinterpret_cast<uint8_t*>(&frame_data)) != HAL_OK) return ;
        if (rx.RTR != CAN_RTR_DATA || rx.IDE != CAN_ID_STD || rx.DLC != 8) return ;

        serial_to_can.write_frame(rx.StdId, frame_data);
        HAL_GPIO_TogglePin(LED_RX_GPIO_Port, LED_RX_Pin);

    } else { //扩展帧

        CAN_RxHeaderTypeDef rx;
        SerialToCan::FrameData frame_data;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, reinterpret_cast<uint8_t*>(&frame_data)) != HAL_OK) return ;
        if (rx.RTR != CAN_RTR_DATA || rx.IDE != CAN_ID_EXT || rx.DLC != 10) return ;

        serial_to_can.write_frame(rx.ExtId, frame_data);
        HAL_GPIO_TogglePin(LED_RX_GPIO_Port, LED_RX_Pin);

    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (id_type == SerialToCan::IDType::STD) { //标准帧

        CAN_RxHeaderTypeDef rx;
        SerialToCan::FrameData frame_data;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx, reinterpret_cast<uint8_t*>(&frame_data)) != HAL_OK) return ;
        if (rx.RTR != CAN_RTR_DATA || rx.IDE != CAN_ID_STD || rx.DLC != 10) return ;

        serial_to_can.write_frame(rx.StdId, frame_data);
        HAL_GPIO_TogglePin(LED_RX_GPIO_Port, LED_RX_Pin);

    } else { //扩展帧

        CAN_RxHeaderTypeDef rx;
        SerialToCan::FrameData frame_data;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx, reinterpret_cast<uint8_t*>(&frame_data)) != HAL_OK) return ;
        if (rx.RTR != CAN_RTR_DATA || rx.IDE != CAN_ID_EXT || rx.DLC != 10) return ;

        serial_to_can.write_frame(rx.ExtId, frame_data);
        HAL_GPIO_TogglePin(LED_RX_GPIO_Port, LED_RX_Pin);

    }
}

void Can_Write(CAN_HandleTypeDef *hcan, uint32_t id, SerialToCan::FrameData data)
{
    if (id_type == SerialToCan::IDType::STD) { //标准帧

        CAN_TxHeaderTypeDef tx;
        tx.StdId = id;
        tx.IDE = CAN_ID_STD;
        tx.RTR = CAN_RTR_DATA;
        tx.TransmitGlobalTime = DISABLE;
        tx.DLC = 8;
        uint32_t mail;
        HAL_CAN_AddTxMessage(hcan, &tx, reinterpret_cast<uint8_t*>(&data), &mail);

    } else { //扩展帧

        CAN_TxHeaderTypeDef tx;
        tx.StdId = id;
        tx.IDE = CAN_ID_EXT;
        tx.RTR = CAN_RTR_DATA;
        tx.TransmitGlobalTime = DISABLE;
        tx.DLC = 8;
        uint32_t mail;
        HAL_CAN_AddTxMessage(hcan, &tx, reinterpret_cast<uint8_t*>(&data), &mail);

    }
}


void CanInit()
{
    SetCanFifo0Filter();
    SetCanFifo1Filter();

    HAL_CAN_Start(&hcan);
    //开启Can接收消息中断
    __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    serial_port.OnHAL_UARTEx_RxEventCallback(huart, size);
}

void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port.OnHAL_UART_TxCpltCallback(huart);
}

void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port.OnHAL_UART_RxCpltCallback(huart);
}

void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    serial_port.OnHAL_UART_ErrorCallback(huart);
}

void MX_CAN_1M_Init()
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_CAN_500K_Init()
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 8;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_CAN_250K_Init()
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 16;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_CAN_100K_Init()
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 40;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
}


void RobotInit()
{
    MX_GPIO_Init();
    //读取拨码开关状态
    GPIO_PinState sw3_status = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
    GPIO_PinState sw2_status = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
    GPIO_PinState sw1_status = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);

    if (sw1_status) { //开启标准帧
        id_type = SerialToCan::IDType::STD;
    } else { //开启扩展帧
        id_type = SerialToCan::IDType::EXT;
    }

    MX_DMA_Init();

    //选择CAN波特率
    if (sw2_status && sw3_status) {
        MX_CAN_1M_Init();
    } else if (!sw2_status && sw3_status) {
        MX_CAN_500K_Init();
    } else if (sw2_status && !sw3_status) {
        MX_CAN_250K_Init();
    } else if (!sw2_status && !sw3_status) {
        MX_CAN_100K_Init();
    }

    MX_USART1_UART_Init();

    serial_to_can.set_id_type(id_type);
    serial_port.start();
    CanInit();
}

void RobotMain()
{
    while (true) {
        uint32_t id;
        SerialToCan::FrameData frame_data;
        std::size_t recv_bytes = serial_to_can.read_frame(&id, &frame_data);
        if (recv_bytes == 0) continue;

        Can_Write(&hcan, id, frame_data);
        HAL_GPIO_TogglePin(LED_TX_GPIO_Port, LED_TX_Pin);
    }
}