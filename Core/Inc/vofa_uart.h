#ifndef VOFA_UART_H
#define VOFA_UART_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

extern uint8_t heart_pending;
extern HAL_StatusTypeDef uart_status;

void UART_SendData(uint32_t red, uint32_t ir, int32_t red_pulse, int32_t ir_pulse, 
                   uint8_t beat_detected, uint32_t heart_rate_bpm, 
                   uint8_t heart_rate_valid, float SPO2_est, uint8_t spo2_valid);












#endif
