
#ifndef MAX30102_H
#define MAX30102_H

#include "stm32f1xx_hal.h"

/* Startup recovery diagnostics live in RAM: inspect these, not live I2C SRx.
 * stage names identify the last operation if recovery fails. */
typedef enum {
  MAX30102_RECOVERY_NOT_RUN = 0,
  MAX30102_RECOVERY_GUARD,
  MAX30102_RECOVERY_RELEASE_SCL,
  MAX30102_RECOVERY_RELEASE_SDA,
  MAX30102_RECOVERY_SDA_LOW,
  MAX30102_RECOVERY_SCL_LOW,
  MAX30102_RECOVERY_SCL_HIGH,
  MAX30102_RECOVERY_SDA_HIGH,
  MAX30102_RECOVERY_REINIT,
  MAX30102_RECOVERY_VERIFY_IDLE,
  MAX30102_RECOVERY_DONE
} MAX30102_RecoveryStage;

typedef struct {
  uint32_t attempts;
  HAL_StatusTypeDef first_init_status;
  HAL_StatusTypeDef bus_status;
  HAL_StatusTypeDef retry_init_status;
  MAX30102_RecoveryStage stage;
  uint32_t error_before;
  uint32_t sr2_before;
  uint32_t clock_pulses;
  uint8_t scl_before;
  uint8_t sda_before;
  uint8_t scl_after;
  uint8_t sda_after;
} MAX30102_RecoveryDiag;

extern volatile MAX30102_RecoveryDiag max30102_recovery;



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
/* Startup only, before the scheduler: at most one bus recovery and one retry.
 * Requires working HAL tick, enabled IRQs and exclusive I2C1 ownership.
 * Recovery is specific to this board: PB6=SCL, PB7=SDA, external pull-ups. */
HAL_StatusTypeDef MAX30102_Init(void);
HAL_StatusTypeDef MAX30102_ReadSample(uint32_t *red_raw,uint32_t *ir_raw);
HAL_StatusTypeDef MAX30102_DataReady(uint8_t *ready);

#endif

