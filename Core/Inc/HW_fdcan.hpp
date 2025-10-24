/**
 *******************************************************************************
 * @file      :HW_fdcan.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      yyyy-mm-dd      <author>        1。<note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HW_FDCAN_H_
#define _HW_FDCAN_H_
/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h" //

#ifdef __cplusplus
}
#endif
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


void FdcanFilterInit(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);

void FdcanSendMsg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id,
                  uint8_t len);


#endif /* _HW_FDCAN_H_ */
