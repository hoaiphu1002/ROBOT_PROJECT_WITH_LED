/*
 * can_treceive.c
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
#include "rfid.h"
#include "can_topic.h"
#include "ultrasonic_sensor.h"
#include "BN055_IT.h"
#include "can_send_noblocking.h"
//khai báo mở rộng, không định nghĩa lại
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbox;
extern uint8_t txData[8];
extern uint8_t rxData[8];
extern MQ135_HandleTypeDef mq135;

extern volatile uint32_t can_tx_count ;
extern volatile uint32_t can_rx_count ;




#define ENABLE_DEBUG_CAN 0  // bật = 1 nếu muốn in gói CAN qua UART

static uint32_t imu_reset_time = 0;
static uint8_t imu_recover_fail_count = 0;

extern volatile uint8_t bno055_need_reset;
extern volatile uint8_t BNO055_I2C_Done;
extern volatile uint8_t BNO055_I2C_Error;


void BNO055_CheckAndRecover(void)
{
    static BNO055_Sensors_t last_data;       // dữ liệu lần trước
    static uint32_t last_update_time = 0;    // thời điểm cập nhật cuối
    static uint8_t first_run = 1;

    // --- 1. Watchdog dữ liệu ---
    BNO055_Sensors_t current;
    ReadData(&current, SENSOR_EULER); // hoặc SENSOR_QUATERNION tùy bạn dùng

    if (first_run) {
        last_data = current;
        last_update_time = HAL_GetTick();
        first_run = 0;
    } else {
        if (memcmp(&current, &last_data, sizeof(BNO055_Sensors_t)) != 0) {
            // dữ liệu thay đổi -> update lại
            last_data = current;
            last_update_time = HAL_GetTick();
        } else if (HAL_GetTick() - last_update_time > 200) {
            // dữ liệu không đổi >200ms → coi như treo
            printf("⚠️ IMU data frozen, trigger reset\r\n");
            bno055_need_reset = 1;
        }
    }

    // --- 2. Thực hiện reset khi có lỗi ---
    if (bno055_need_reset) {
        if (HAL_GetTick() - imu_reset_time > 2000) {  // tránh reset liên tục
            imu_reset_time = HAL_GetTick();
            printf("⚠️ IMU recover attempt...\r\n");

            // 1. Reset I2C bus
            I2C_ManualBusRecovery();

            // 2. Reset IMU
            ResetBNO055();

            // 3. Đợi SYS_STATUS = 5 (Fusion Algorithm Running)
            uint8_t sys_status = 0;
            uint32_t start = HAL_GetTick();
            do {
                BNO055_IT_Read(P_BNO055, SYS_STATUS_ADDR, &sys_status, 1);
                HAL_Delay(20);
            } while (sys_status != 5 && (HAL_GetTick() - start < 3000));

            if (sys_status == 5) {
                imu_recover_fail_count = 0;
                bno055_need_reset = 0;   // ✅ chỉ clear khi thành công
                printf("✅ IMU recovered successfully\r\n");
            } else {
                imu_recover_fail_count++;
                printf("❌ IMU recover failed (%d)\r\n", imu_recover_fail_count);

                if (imu_recover_fail_count >= 3) {
                    printf("🚨 Too many IMU failures → system reset\r\n");
                    NVIC_SystemReset();
                }
            }

            BNO055_I2C_Error = 0;
            BNO055_I2C_Done  = 0;
        }
    }
}


bool BNO055_IsStable(void)
{
    return (HAL_GetTick() - imu_reset_time > 600);  // chờ 1 giây
}


void CAN_Loopback_Test(void)
{
	  uint8_t tmp[8] = { 0x3F, 0xFF, 0x80, 0x77, 0, 0, 0, 0 };
	      memcpy(txData, tmp, sizeof(tmp));
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &TxMailbox) != HAL_OK) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"TX FAIL\r\n", 9, HAL_MAX_DELAY);
        return;
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"TX OK\r\n", 7, HAL_MAX_DELAY);
    }

    HAL_Delay(50);  // Cho frame vào FIFO

HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rxData);

        char buf[64];
        sprintf(buf, "\nRX OK: ID=0x%03X DLC=%lu DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                (unsigned int)RxHeader.StdId,
                RxHeader.DLC,
                rxData[0], rxData[1], rxData[2], rxData[3],rxData[4],rxData[5],rxData[6],rxData[7]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}


HAL_StatusTypeDef CAN_SendString(uint16_t stdId, const char *str)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t TxMailbox;
    size_t len = strlen(str);
    if (len > 8) len = 8;
    memcpy(TxData, str, len);

    TxHeader.StdId = stdId;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

    if (status != HAL_OK) {
        char err[] = "CAN FRAME SEND FAIL\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
    }

    return status;
}
HAL_StatusTypeDef CAN_SendTopicData(uint16_t topic_id, uint8_t *data, uint8_t len)
{
    uint32_t TxMailbox;
    HAL_StatusTypeDef status;
    uint32_t start = HAL_GetTick();
    const uint32_t timeout_ms = 3;   // thời gian retry tối đa (5ms)

    if (len > 8) len = 8;

    TxHeader.StdId = topic_id;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

#if ENABLE_DEBUG_CAN
    char log[128];
    int offset = snprintf(log, sizeof(log), "\r\n[CAN TX] Topic: 0x%03X | Len: %d | Data:", topic_id, len);
    for (uint8_t i = 0; i < len; i++) {
        offset += snprintf(log + offset, sizeof(log) - offset, " %02X", data[i]);
    }
    snprintf(log + offset, sizeof(log) - offset, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)log, strlen(log), 1);
#endif

    can_tx_count++;

    // --- Retry loop ---
    do {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
            status = CAN_SendNonBlocking(&hcan1, &TxHeader, data, &TxMailbox);
            if (status == HAL_OK) {
                return HAL_OK;  // gửi thành công
            }
        }
        // Nếu mailbox vẫn bận, chờ 1 tick rồi thử lại
    } while ((HAL_GetTick() - start) < timeout_ms);

    // Nếu hết thời gian vẫn không gửi được
    return HAL_TIMEOUT;
    if (status==HAL_TIMEOUT){
    	 // Reset CAN

//        HAL_CAN_Stop(&hcan1);
//        HAL_CAN_DeInit(&hcan1);
//        HAL_CAN_Start(&hcan1);
//        HAL_CAN_ActivateNotification(&hcan1,
//            CAN_IT_ERROR_WARNING |
//            CAN_IT_ERROR_PASSIVE |
//            CAN_IT_BUSOFF |
//            CAN_IT_LAST_ERROR_CODE |
//            CAN_IT_ERROR);
        NVIC_SystemReset();

    }
}

extern MQ135_HandleTypeDef mq135;
extern uint8_t currentUID[UID_LEN];  // từ rfid.c
extern uint32_t debug_timer;
extern uint32_t imu_timer;
extern volatile uint8_t timer10ms_flag ;


void Send_All_SensorData_CAN(void)
{
	static uint32_t last_us_trigger_time ;

    if (timer10ms_flag) {
        timer10ms_flag = 0;
//        BNO055_SendEulerCAN();
        if (BNO055_IsStable()) {
              BNO055_SendEulerCAN();
          } else {
              // bỏ qua dữ liệu trong lúc IMU đang ổn định lại
              HAL_I2C_DeInit(&bno_i2c);
              HAL_Delay(5);
              HAL_I2C_Init(&bno_i2c);

          }
    }

    if (HAL_GetTick() - debug_timer >= 20) {
        BNO055_PrintEulerDebug();
        debug_timer = HAL_GetTick();
    }

    if (HAL_GetTick() - last_us_trigger_time >= 400) {
            US01_TriggerAll_Sequential();      // Blocking đo 4 cảm biến
            PrintAllDistances();               // UART in khoảng cách
            US01_SendAllDistances_CAN();       // Gửi qua CAN
            last_us_trigger_time = HAL_GetTick();
        }



    static uint32_t last_mq135_time = 0;
    if (HAL_GetTick() - last_mq135_time >= 1000) {
        MQ135_Send_CAN(&mq135, 25.0f, 50.0f, &huart1, TOPIC_ID_MQ135);  // Dùng hàm DMA mới
        last_mq135_time = HAL_GetTick();
    }
    checkRFIDAndControlRelay();
    static uint32_t last_can_tx = 0;
    static uint8_t can_fail_count = 0;

    if (HAL_GetTick() - last_can_tx >= 300) { // kiểm tra mỗi giây
        if (can_tx_count == 0 && can_rx_count == 0) {
            can_fail_count++;
        } else {
            can_fail_count = 0;
        }

        if (can_fail_count >= 3) { // 3 giây không có hoạt động
            char msg[] = "⚠️ CAN watchdog triggered, reinit...\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

            // Reset lại CAN1 hoàn chỉnh
            HAL_CAN_Stop(&hcan1);
            HAL_CAN_DeInit(&hcan1);
            HAL_CAN_Init(&hcan1);
            HAL_CAN_Start(&hcan1);


            // Bật lại interrupt
            HAL_CAN_ActivateNotification(&hcan1,
                CAN_IT_ERROR_WARNING |
                CAN_IT_ERROR_PASSIVE |
                CAN_IT_BUSOFF |
                CAN_IT_LAST_ERROR_CODE |
                CAN_IT_ERROR);


            can_fail_count = 0;
        }

        can_tx_count = 0;
        can_rx_count = 0;
        last_can_tx = HAL_GetTick();
    }

}
