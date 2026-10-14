#include "main.h"
#include "stm32f1xx_hal.h"

#include "max30102.h"



uint8_t first_config_fail_reg = 0xFFU;       
HAL_StatusTypeDef first_config_fail_status = HAL_OK;
uint32_t first_config_fail_state = 0;        
uint32_t first_config_fail_sr2 = 0;         

extern I2C_HandleTypeDef hi2c1;

volatile MAX30102_RecoveryDiag max30102_recovery = {0};

#define MAX30102_SCL_PIN GPIO_PIN_6
#define MAX30102_SDA_PIN GPIO_PIN_7
#define MAX30102_LINE_WAIT_MS 5U

static HAL_StatusTypeDef MAX30102_WaitLine(uint16_t pin, GPIO_PinState level)
{
  uint32_t wait_ms;
  for (wait_ms = 0; wait_ms < MAX30102_LINE_WAIT_MS; ++wait_ms)
  {
    if (HAL_GPIO_ReadPin(GPIOB, pin) == level) return HAL_OK;
    HAL_Delay(1);
  }
  return HAL_GPIO_ReadPin(GPIOB, pin) == level ? HAL_OK : HAL_TIMEOUT;
}

/* Single-controller bus, only used after a failed startup transaction.
 * SDA stuck low: up to 9 clocks (NXP UM10204, bus clear).
 * Lines released but BUSY stuck: ST ES096 rev15 section 2.8.7 sequence.
 * Every driven high is open-drain release, never push-pull high. */
static HAL_StatusTypeDef MAX30102_RecoverStartupBus(void)
{
  GPIO_InitTypeDef gpio = {0};
  HAL_StatusTypeDef status = HAL_TIMEOUT;
  HAL_StatusTypeDef init_status;
  uint32_t wait_ms;

  ++max30102_recovery.attempts;
  max30102_recovery.stage = MAX30102_RECOVERY_GUARD;
  max30102_recovery.clock_pulses = 0;
  if (hi2c1.Instance != I2C1 || hi2c1.Lock != HAL_UNLOCKED ||
      hi2c1.State != HAL_I2C_STATE_READY || __get_IPSR() != 0U ||
      __get_PRIMASK() != 0U)
  {
    max30102_recovery.bus_status = HAL_BUSY;
    return HAL_BUSY;
  }

  /* Capture only after transfer failure, before touching the peripheral. */
  max30102_recovery.error_before = hi2c1.ErrorCode;
  max30102_recovery.sr2_before = hi2c1.Instance->SR2;
  max30102_recovery.scl_before = (uint8_t)HAL_GPIO_ReadPin(GPIOB, MAX30102_SCL_PIN);
  max30102_recovery.sda_before = (uint8_t)HAL_GPIO_ReadPin(GPIOB, MAX30102_SDA_PIN);

  __HAL_I2C_DISABLE(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, MAX30102_SCL_PIN | MAX30102_SDA_PIN, GPIO_PIN_SET);
  gpio.Pin = MAX30102_SCL_PIN | MAX30102_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_Delay(1);

  max30102_recovery.stage = MAX30102_RECOVERY_RELEASE_SCL;
  if (MAX30102_WaitLine(MAX30102_SCL_PIN, GPIO_PIN_SET) != HAL_OK) goto restore;

  /* A reset may have interrupted the sensor in the middle of a byte. */
  max30102_recovery.stage = MAX30102_RECOVERY_RELEASE_SDA;
  while (HAL_GPIO_ReadPin(GPIOB, MAX30102_SDA_PIN) == GPIO_PIN_RESET &&
         max30102_recovery.clock_pulses < 9U)
  {
    HAL_GPIO_WritePin(GPIOB, MAX30102_SCL_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, MAX30102_SCL_PIN, GPIO_PIN_SET);
    ++max30102_recovery.clock_pulses;
    max30102_recovery.stage = MAX30102_RECOVERY_RELEASE_SCL;
    if (MAX30102_WaitLine(MAX30102_SCL_PIN, GPIO_PIN_SET) != HAL_OK) goto restore;
    HAL_Delay(1);
    max30102_recovery.stage = MAX30102_RECOVERY_RELEASE_SDA;
  }
  if (MAX30102_WaitLine(MAX30102_SDA_PIN, GPIO_PIN_SET) != HAL_OK) goto restore;

  /* ES096: force both line transitions, checking the actual input levels.
   * This produces START then STOP; do not issue any sensor data write here. */
  max30102_recovery.stage = MAX30102_RECOVERY_SDA_LOW;
  HAL_GPIO_WritePin(GPIOB, MAX30102_SDA_PIN, GPIO_PIN_RESET);
  if (MAX30102_WaitLine(MAX30102_SDA_PIN, GPIO_PIN_RESET) != HAL_OK) goto restore;
  HAL_Delay(1);
  max30102_recovery.stage = MAX30102_RECOVERY_SCL_LOW;
  HAL_GPIO_WritePin(GPIOB, MAX30102_SCL_PIN, GPIO_PIN_RESET);
  if (MAX30102_WaitLine(MAX30102_SCL_PIN, GPIO_PIN_RESET) != HAL_OK) goto restore;
  HAL_Delay(1);
  max30102_recovery.stage = MAX30102_RECOVERY_SCL_HIGH;
  HAL_GPIO_WritePin(GPIOB, MAX30102_SCL_PIN, GPIO_PIN_SET);
  if (MAX30102_WaitLine(MAX30102_SCL_PIN, GPIO_PIN_SET) != HAL_OK) goto restore;
  HAL_Delay(1);
  max30102_recovery.stage = MAX30102_RECOVERY_SDA_HIGH;
  HAL_GPIO_WritePin(GPIOB, MAX30102_SDA_PIN, GPIO_PIN_SET);
  if (MAX30102_WaitLine(MAX30102_SDA_PIN, GPIO_PIN_SET) != HAL_OK) goto restore;
  HAL_Delay(1);
  status = HAL_OK;

restore:
  /* Also restore pin mode on failure. Keep the failing stage for Watch. */
  HAL_GPIO_WritePin(GPIOB, MAX30102_SCL_PIN | MAX30102_SDA_PIN, GPIO_PIN_SET);
  gpio.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(GPIOB, &gpio);
  if (status == HAL_OK) max30102_recovery.stage = MAX30102_RECOVERY_REINIT;
  /* The local HAL init asserts/clears SWRST, restores timing, then sets PE. */
  init_status = HAL_I2C_Init(&hi2c1);
  if (status == HAL_OK) status = init_status;
  if (status == HAL_OK)
  {
    max30102_recovery.stage = MAX30102_RECOVERY_VERIFY_IDLE;
    status = HAL_BUSY;
    for (wait_ms = 0; wait_ms < MAX30102_LINE_WAIT_MS; ++wait_ms)
    {
      if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET)
      {
        status = HAL_OK;
        break;
      }
      HAL_Delay(1);
    }
  }
  max30102_recovery.scl_after = (uint8_t)HAL_GPIO_ReadPin(GPIOB, MAX30102_SCL_PIN);
  max30102_recovery.sda_after = (uint8_t)HAL_GPIO_ReadPin(GPIOB, MAX30102_SDA_PIN);
  max30102_recovery.bus_status = status;
  if (status == HAL_OK) max30102_recovery.stage = MAX30102_RECOVERY_DONE;
  return status;
}


HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
      &hi2c1,                            /* 使用 CubeMX 配好的 I2C1 外设 */
      (uint16_t)(MAX30102_ADDR_7BIT << 1),/* HAL 要左移一位后的设备地址 */
      reg,                               /* 传感器内部的寄存器地址 */
      I2C_MEMADD_SIZE_8BIT,              /* 寄存器地址本身占 1 字节 */
      &value,                            /* 待写数据的地址；函数从这里取值 */
      1U,                                /* 这次只写 1 个数据字节 */
      100U);                             /* 最多等待 100 ms，防止总线故障时一直卡住 */

  /* config_status 会被后面的写入覆盖；这里仅保存第一次失败的现场。
   * reg 指出失败发生在哪个寄存器；State 区分 HAL 句柄是否仍在传输；
   * SR2 的 bit 1 是硬件 BUSY 标志，置 1 表示外设认为总线还在忙。 */
  if (status != HAL_OK && first_config_fail_reg == 0xFFU)
  {
    first_config_fail_reg = reg;
    first_config_fail_status = status;
    first_config_fail_state = (uint32_t)hi2c1.State;
    first_config_fail_sr2 = hi2c1.Instance->SR2;
  }
  return status;
}

/* 从 MAX30102 的一个寄存器开始读取 size 个字节。
 * 为什么包装 HAL_I2C_Mem_Read：读指针和读 FIFO 都走同一条 I2C 总线，
 * 地址、寄存器地址宽度和超时相同；调用处只需说明读哪里、放哪里、读多少。
 * 对 FIFO_DATA(0x07) 读 6 字节时，芯片会连续送出同一组的红光 3 字节、
 * 红外光 3 字节；不要拆成 6 次独立的单字节读取。
 * reg 是寄存器地址；data 是接收缓冲区地址；size 是接收字节数。
 * 返回 HAL_OK 才表示这次传输成功。 */
HAL_StatusTypeDef MAX30102_ReadBytes(uint8_t reg, uint8_t *data, uint16_t size)
{
  return HAL_I2C_Mem_Read(
      &hi2c1,                             /* CubeMX 初始化的 I2C1 句柄 */
      (uint16_t)(MAX30102_ADDR_7BIT << 1),/* HAL 使用左移一位的设备地址 */
      reg,                                /* 先告诉传感器从哪个寄存器读 */
      I2C_MEMADD_SIZE_8BIT,               /* 寄存器地址长度为 1 字节 */
      data,                               /* 接收数据的内存地址 */
      size,                               /* 本次连续接收的字节数 */
      100U);                              /* 超时上限 100 ms */
}

static HAL_StatusTypeDef MAX30102_ConfigureOnce(void)
{ HAL_StatusTypeDef status;
  status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG,0x40);
  if(status != HAL_OK)return status;
  HAL_Delay(10); 
  /* TODO M2-A-2：其余八次寄存器写入写在这里 */
  status = MAX30102_WriteReg(MAX30102_REG_FIFO_CONFIG,0x10);
  if(status != HAL_OK)return status;
  status = MAX30102_WriteReg(MAX30102_REG_SPO2_CONFIG,0x27);
  if(status != HAL_OK)return status;
  status = MAX30102_WriteReg(MAX30102_REG_LED1_PA,0x1F);
  if(status != HAL_OK)return status;
  status = MAX30102_WriteReg(MAX30102_REG_LED2_PA,0x1F);
  if(status != HAL_OK)return status;
  status = MAX30102_WriteReg(MAX30102_REG_FIFO_WR_PTR,0x00);
  if(status != HAL_OK)return status;
  status = MAX30102_WriteReg(MAX30102_REG_OVF_COUNTER,0x00);
  if(status != HAL_OK)return status;
  status = MAX30102_WriteReg(MAX30102_REG_FIFO_RD_PTR,0x00);
  if(status != HAL_OK)return status;
  status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG,0x03);
  if(status != HAL_OK)return status;
 
  return HAL_OK;
}

HAL_StatusTypeDef MAX30102_Init(void)
{
  HAL_StatusTypeDef status = MAX30102_ConfigureOnce();
  max30102_recovery.first_init_status = status;
  if (status == HAL_OK) return HAL_OK;

  /* Do not hide missing-device NACKs or retry arbitrary FIFO reads/writes.
   * Only startup BUSY/TIMEOUT takes the recovery path, once per Init call. */
  if (status != HAL_BUSY && status != HAL_TIMEOUT &&
      (hi2c1.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == 0U) return status;

  max30102_recovery.retry_init_status = HAL_ERROR;
  status = MAX30102_RecoverStartupBus();
  if (status != HAL_OK) return status;

  status = MAX30102_ConfigureOnce();
  max30102_recovery.retry_init_status = status;
  return status;
}

HAL_StatusTypeDef MAX30102_ReadSample(uint32_t *red_raw,uint32_t *ir_raw)
{
    uint8_t fifo_bytes[6] = {0};                 /* 一组样本：红光 3 字节、红外光 3 字节 */
    HAL_StatusTypeDef fifo_status = MAX30102_ReadBytes(MAX30102_REG_FIFO_DATA,fifo_bytes,6u);
      if(fifo_status == HAL_OK)
      {
        *red_raw = ((uint32_t)fifo_bytes[0] << 16)
                 |((uint32_t)fifo_bytes[1] << 8)
                 |((uint32_t)fifo_bytes[2]);
        *ir_raw = ((uint32_t)fifo_bytes[3] << 16)
                 |((uint32_t)fifo_bytes[4] << 8)
                 |((uint32_t)fifo_bytes[5]);
        *red_raw &= 0x3FFFFU;
        *ir_raw &= 0x3FFFFU;
      }
      return fifo_status;
      
}
HAL_StatusTypeDef MAX30102_DataReady(uint8_t *ready)
{   
    uint8_t fifo_wr_ptr = 0;                     /* 传感器下一次要写的 FIFO 位置 */
    uint8_t fifo_rd_ptr = 0;                     /* MCU 下一次要读的 FIFO 位置 */
    *ready = 0;
    HAL_StatusTypeDef wr_status = HAL_ERROR;
    HAL_StatusTypeDef rd_status = HAL_ERROR;
    
    wr_status = MAX30102_ReadBytes(MAX30102_REG_FIFO_WR_PTR, &fifo_wr_ptr, 1U);
    if(wr_status != HAL_OK)return wr_status;
    rd_status = MAX30102_ReadBytes(MAX30102_REG_FIFO_RD_PTR, &fifo_rd_ptr, 1U);
    if(rd_status != HAL_OK)return rd_status;
    if(fifo_wr_ptr != fifo_rd_ptr)
    {
        *ready = 1;
        
    }
    return HAL_OK;
}
