#include "main.h"
#include "stm32f1xx_hal.h"

#include "max30102.h"



uint8_t first_config_fail_reg = 0xFFU;       /* 0xFF = 九次配置写入均未报告失败 */
HAL_StatusTypeDef first_config_fail_status = HAL_OK;
uint32_t first_config_fail_state = 0;        /* 第一次失败时 HAL I2C 句柄的状态 */
uint32_t first_config_fail_sr2 = 0;          /* 第一次失败时 I2C1 状态寄存器 SR2 */

extern I2C_HandleTypeDef hi2c1;


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

HAL_StatusTypeDef MAX30102_Init(void)
{ HAL_StatusTypeDef status;
  status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG,0x40);
  if(status != HAL_OK)return status;
  HAL_Delay(10); /* 等传感器完成复位；它不会自动执行复位，仍需完成上面的 TODO。 */
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
