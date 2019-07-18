#ifndef __LORA_PTOP_133EE85_H
#define __LORA_PTOP_133EE85_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/

#include  "radio.h"
#include  "sx1276.h"
#include  "eml3047.h"

enum {
    OP_MUTEX_LOCK             = 0,
    OP_MUTEX_UNLOCK              ,
};

#define    WATIE_UNBLOCK      0
#define    WATIE_BLOCK        (portMAX_DELAY)


#define RX_SF          12
#define TX_SF          12
#define TX_PWR         20 

/**
 * @brief  SX1278_ptop_config 
 * @param  sync_byte:0x12
 * @param  power:20
 * @retval None
 */     
void LORA_ptop_config(uint8_t sync_byte, int8_t power);
void LORA_ptop_ModifySFRxValue(uint8_t value);
/**
 * @brief  LORA_ptop_SetInRxMode 
 * @param  None, Channel:475500000 power:20
 * @retval None
 */
void LORA_ptop_SetInRxMode(uint8_t op_lock);
/**
 * @brief  LORA_ptop_SendMsg 
 * @param  pchar 
 * @param  len 
 * @retval None
 */
void LORA_ptop_SendMsg(uint8_t sf, int8_t pwr, uint8_t* pchar, uint32_t len);
/**
 * @brief  LORA_ptop_ReceiveMsg 
 * @param  ppchar, pointer to buf (in malloc)
 * @param  len,  the buf length
 * @param  rssi
 * @param  snr
 * @param  block_wait,  WATIE_UNBLOCK, WATIE_BLOCK, or set the wait_time(tick time)
 * @retval void*, the message pointer in malloc, must be freed after use_up
 */
void* LORA_ptop_ReceiveMsg( uint8_t **ppchar, uint16_t* len, int16_t* rssi, int8_t* snr, uint32_t block_wait);

/**
 * @brief  LORA_ptop_SetSleep 
 * @param  None 
 * @retval None
 */
void LORA_ptop_SetSleep(void);

#ifdef __cplusplus
}
#endif

#endif

