
#ifndef MAX30102_H
#define MAX30102_H

#include "stm32f1xx_hal.h"



#define MAX30102_ADDR_7BIT   0x57U
#define MAX30102_PART_ID_REG 0xFFU
/* M2-A：这些地址来自 MAX30102 数据手册的 Register Map。
 * 左边三个是 FIFO 的写指针、溢出计数、读指针；启动新一轮采样前都要清零。 */
#define MAX30102_REG_FIFO_WR_PTR 0x04U
#define MAX30102_REG_OVF_COUNTER 0x05U
#define MAX30102_REG_FIFO_RD_PTR 0x06U
#define MAX30102_REG_FIFO_DATA   0x07U
#define MAX30102_REG_FIFO_CONFIG 0x08U
#define MAX30102_REG_MODE_CONFIG 0x09U
#define MAX30102_REG_SPO2_CONFIG 0x0AU
#define MAX30102_REG_LED1_PA     0x0CU   /* LED1 = 红光 */
#define MAX30102_REG_LED2_PA     0x0DU   /* LED2 = 红外光 */








HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef MAX30102_ReadBytes(uint8_t reg, uint8_t *data, uint16_t size);
HAL_StatusTypeDef MAX30102_Init(void);
HAL_StatusTypeDef MAX30102_ReadSample(uint32_t *red_raw,uint32_t *ir_raw);
HAL_StatusTypeDef MAX30102_DataReady(uint8_t *ready);

#endif

