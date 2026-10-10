#include "vofa_uart.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>  /* snprintf：把整数格式化为串口可发送的文本 */

uint8_t sample_processed = 5;
 uint8_t heart_pending = 0;
HAL_StatusTypeDef uart_status = HAL_ERROR;   /* 最近一次串口发送状态，供 Watch 观察 */
extern UART_HandleTypeDef huart1;

void UART_SendData(uint32_t red, uint32_t ir, int32_t red_pulse, int32_t ir_pulse, uint8_t beat_detected, uint32_t heart_rate_bpm, uint8_t heart_rate_valid, float spo2_est, uint8_t spo2_valid)
{
  static uint32_t time = 0;
  static uint8_t time_count = 0;
  char line[96];
  uint16_t heartbeat_marker = 0;
  float spo2_tobe_sent = 0;
  if(beat_detected)
  {
    heart_pending = 1;
  }
  if(heart_pending ==1)
  {
    heartbeat_marker = 10000U;
  }
  else
  {
    heartbeat_marker = 0U;
  }
  if(spo2_valid)
  {
    spo2_tobe_sent = spo2_est;
  }
  else
  {
    spo2_tobe_sent = 0;
  }
 if(time_count == 0)
 {
  time = HAL_GetTick();
  time_count++;
 }
  

  uint32_t heart_rate_to_sent = heart_rate_valid ? heart_rate_bpm : 0U;
  if(HAL_GetTick() - time >= 50 && time_count == 1)
  {
    int len = snprintf(line, sizeof(line), "%lu,%lu,%ld,%ld,%lu,%lu,%.1f,%d\r\n",
                     (unsigned long)red, (unsigned long)ir,
                     (long)red_pulse, (long)ir_pulse,
                     (unsigned long)heartbeat_marker,
                     (unsigned long)heart_rate_to_sent,
                     spo2_tobe_sent,
                     spo2_valid);
    if (len > 0 && len < (int)sizeof(line))
    {
      uart_status = HAL_UART_Transmit(&huart1, (uint8_t *)line,
                                    (uint16_t)len, 100U);
      if(uart_status == HAL_OK)heart_pending = 0;
    }
    time = HAL_GetTick();
  }
}






