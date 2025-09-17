/*
 * can_send_noblocking.c
 *
 *  Created on: Sep 16, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "can_send_noblocking.h"
//HAL_StatusTypeDef CAN_SendNonBlocking(CAN_HandleTypeDef *hcan,
//                                      CAN_TxHeaderTypeDef *header,
//                                      uint8_t *data,
//                                      uint32_t *mailbox)
//{
//    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
//        return HAL_CAN_AddTxMessage(hcan, header, data, mailbox);
//    } else {
//        return HAL_BUSY;
//    }
//}

HAL_StatusTypeDef CAN_SendNonBlocking(CAN_HandleTypeDef *hcan,
                                      CAN_TxHeaderTypeDef *header,
                                      uint8_t *data,
                                      uint32_t *mailbox)
{
    // 1. Nếu còn slot trống → gửi luôn
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        return HAL_CAN_AddTxMessage(hcan, header, data, mailbox);
    }

    // 2. Nếu đầy → hủy hết pending để tránh nghẽn
    HAL_CAN_AbortTxRequest(hcan, 0x7);  // 0x7 = abort mailbox 0,1,2

    // 3. Thử gửi lại frame mới (retry tối đa 3 lần)
    for (int retry = 0; retry < 3; retry++) {
        if (HAL_CAN_AddTxMessage(hcan, header, data, mailbox) == HAL_OK) {
            return HAL_OK;
        }
    }

    // Nếu sau retry vẫn lỗi thì trả về HAL_ERROR
    return HAL_ERROR;
}


