/*
 * can_receive.c
 *
 *  Created on: Jul 11, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "can_receive.h"
#include "stm32f4xx_hal_uart.h"
#include "main.h"
#include "RC522.h"
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include "MQ135.h"
#include "liquidcrystal_i2c.h"
#include "BNO055_STM32.h"
#include "display.h"

//  Chỉ khai báo extern, KHÔNG định nghĩa lại
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbox;
extern uint8_t txData[8];
extern uint8_t rxData[8];
extern volatile uint8_t can_rx_flag;
extern volatile uint32_t can_rx_count ;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//    HAL_UART_Transmit(&huart1, (uint8_t*)"INTERRUPT OK\r\n", 15, HAL_MAX_DELAY);
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxData) ;
                  can_rx_flag = 1;  // báo về main xử lý
                  can_rx_count++;  // tăng biến đếm khi nhận
}

void CAN_ReceiveAndPrint(void)
{
//    uint8_t RxData[8];
    char buffer[64];

//    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rxData) ;

    if (RxHeader.DLC == 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"⚠️ EMPTY FRAME RECEIVED\r\n", 26, HAL_MAX_DELAY);
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)"✅ CAN RECEIVED\r\n", 17, HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "RX : ID=0x%03lX DLC=%lu DATA=",
             (uint32_t)RxHeader.StdId, (uint32_t)RxHeader.DLC);

    for (uint8_t i = 0; i < RxHeader.DLC; i++) {
        char hex[8];
        sprintf(hex, "%02X ", rxData[i]);
        strcat(buffer, hex);
    }

    strcat(buffer, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    for (int i = 0; i < 4; i++) {
        char dbg[32];
        snprintf(dbg, sizeof(dbg), "Byte[%d] = 0x%02X\r\n", i, rxData[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
    }
    float velocity_f = 0.00f;
            int32_t velocity_i = 0;

            memcpy(&velocity_f, rxData, sizeof(float));
            memcpy(&velocity_i, rxData, sizeof(int32_t)); // nếu cần int thay float

            snprintf(buffer, sizeof(buffer),
                     "\rVelocity: float = %.2f | int = %ld\r\n",
                     velocity_f, (long)velocity_i);

            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void send_velocity_rpdo(uint8_t node, int32_t velocity, bool toggle_cw, uint32_t accel, uint32_t decel) {
    static bool toggle = false;
    uint8_t data[8];

    // 1. Set acceleration & deceleration trước khi gửi velocity
    send_sdo_write_u32(node, 0x6083, 0x00, accel);  // acceleration
    HAL_Delay(2);
    send_sdo_write_u32(node, 0x6084, 0x00, decel);  // deceleration
    HAL_Delay(2);

    // 2. Toggle CW
    uint16_t cw = toggle_cw ? (toggle ? 0x1F : 0x0F) : 0x0F;
    toggle = !toggle;

    // 3. Build RPDO data (CW + velocity)
    data[0] = cw & 0xFF;
    data[1] = (cw >> 8) & 0xFF;

    data[2] = (velocity >> 0) & 0xFF;
    data[3] = (velocity >> 8) & 0xFF;
    data[4] = (velocity >> 16) & 0xFF;
    data[5] = (velocity >> 24) & 0xFF;

    data[6] = 0;
    data[7] = 0;

    // 4. Gửi RPDO
    TxHeader.StdId = 0x200 + node;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
    HAL_Delay(5);
}
int32_t read_actual_velocity(uint8_t nodeId) {
    CAN_TxHeaderTypeDef tx;
    CAN_RxHeaderTypeDef rx;
    uint8_t txData[8] = {0};
    uint8_t rxData[8] = {0};

    // ==== Gửi yêu cầu đọc SDO (0x606C:00) ====
    tx.StdId = 0x600 + nodeId;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    txData[0] = 0x40;      // SDO Upload request
    txData[1] = 0x6C;      // Index LSB (0x606C)
    txData[2] = 0x60;      // Index MSB
    txData[3] = 0x00;      // Subindex = 0
    // txData[4..7] = 0
    HAL_CAN_AddTxMessage(&hcan1, &tx, txData, &TxMailbox);
HAL_Delay(100);
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, rxData);
    // ==== Ghép vận tốc từ Little Endian 4 byte ====
    int32_t velocity = (int32_t)(
        ((uint32_t)rxData[4]) |
        ((uint32_t)rxData[5] << 8) |
        ((uint32_t)rxData[6] << 16) |
        ((uint32_t)rxData[7] << 24)
    );

    return velocity;
}
void send_sdo_write_u32(uint8_t nodeId, uint16_t index, uint8_t subidx, uint32_t value) {
    uint8_t data[8] = {
        0x23,
        index & 0xFF,
        (index >> 8) & 0xFF,
        subidx,
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF
    };

    TxHeader.StdId = 0x600 + nodeId;
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
    HAL_Delay(10);
}
void print_velocity_both_nodes() {
    int32_t v1 = read_actual_velocity(1);
    int32_t v2 = read_actual_velocity(2);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "🔄 Velocities | Node1: %ld  | Node2: %ld\r\n", v1, v2);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // In ra toàn bộ các frame còn trong FIFO
   // while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef rx;
        uint8_t rxData[8];

        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, rxData);

        char log[128];
        snprintf(log, sizeof(log),
            "📥 CAN RX: ID=0x%03lX | DLC=%d | Data= %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            rx.StdId,
            rx.DLC,
            rxData[0], rxData[1], rxData[2], rxData[3],
            rxData[4], rxData[5], rxData[6], rxData[7]);
        HAL_UART_Transmit(&huart1, (uint8_t*)log, strlen(log), HAL_MAX_DELAY);
  //  }
}

