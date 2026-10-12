/* Host-side fault injection against the actual max30102.c implementation.
 * This checks control flow and bounds, not STM32 electrical behavior. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "max30102.h"

I2C_TypeDef test_i2c1;
GPIO_TypeDef test_gpiob;
I2C_HandleTypeDef hi2c1;
uint32_t test_ipsr, test_primask;
extern uint8_t first_config_fail_reg;
extern HAL_StatusTypeDef first_config_fail_status;
extern uint32_t first_config_fail_state, first_config_fail_sr2;

static unsigned writes, reinitializations, gpio_changes, delayed_ms;
static unsigned fail_at, release_after, clock_edges;
static int scl_stuck, sda_stuck, busy_after_reset;
static HAL_StatusTypeDef first_failure, retry_failure, reinit_failure;
static uint8_t written_regs[32], written_values[32];

void HAL_Delay(uint32_t ms)
{
  delayed_ms += ms;
  assert(delayed_ms < 200U); /* Catch runaway recovery loops. */
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  assert(port == GPIOB);
  if (pin == GPIO_PIN_6 && scl_stuck) return GPIO_PIN_RESET;
  if (pin == GPIO_PIN_7 && sda_stuck) return GPIO_PIN_RESET;
  return (port->ODR & pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level)
{
  uint32_t previous = port->ODR;
  if (level == GPIO_PIN_SET) port->ODR |= pin;
  else port->ODR &= ~(uint32_t)pin;
  if (port->mode == GPIO_MODE_OUTPUT_OD && (pin & GPIO_PIN_6) != 0U &&
      (previous & GPIO_PIN_6) == 0U && level == GPIO_PIN_SET && !scl_stuck)
  {
    ++clock_edges;
    if (release_after != 0U && clock_edges >= release_after) sda_stuck = 0;
  }
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
  assert(init->Pin == (GPIO_PIN_6 | GPIO_PIN_7));
  assert(init->Mode == GPIO_MODE_OUTPUT_OD || init->Mode == GPIO_MODE_AF_OD);
  port->mode = init->Mode;
  ++gpio_changes;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *handle)
{
  ++reinitializations;
  assert(test_gpiob.mode == GPIO_MODE_AF_OD);
  handle->State = HAL_I2C_STATE_READY;
  handle->Lock = HAL_UNLOCKED;
  handle->ErrorCode = 0;
  handle->Instance->CR1 = 1;
  handle->Instance->SR2 = busy_after_reset || scl_stuck || sda_stuck ? 2U : 0U;
  return reinit_failure;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
  HAL_StatusTypeDef result = HAL_OK;
  assert(address == (0x57U << 1) && reg_size == 1 && size == 1 && timeout == 100);
  assert(writes < 32U);
  written_regs[writes] = (uint8_t)reg;
  written_values[writes] = *data;
  ++writes;
  if (writes == fail_at) result = first_failure;
  else if (reinitializations != 0U) result = retry_failure;
  if (result == HAL_BUSY || result == HAL_TIMEOUT)
  {
    handle->ErrorCode = HAL_I2C_ERROR_TIMEOUT;
    handle->Instance->SR2 = 2;
  }
  else if (result == HAL_ERROR) handle->ErrorCode = 4; /* NACK */
  return result;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
  (void)handle; (void)address; (void)reg; (void)reg_size; (void)timeout;
  memset(data, 0, size);
  return HAL_OK;
}

static void reset_case(void)
{
  memset(&test_i2c1, 0, sizeof(test_i2c1));
  test_i2c1.CR1 = 1;
  test_gpiob.ODR = GPIO_PIN_6 | GPIO_PIN_7;
  test_gpiob.mode = GPIO_MODE_AF_OD;
  hi2c1.Instance = I2C1;
  hi2c1.State = HAL_I2C_STATE_READY;
  hi2c1.Lock = HAL_UNLOCKED;
  hi2c1.ErrorCode = 0;
  max30102_recovery = (MAX30102_RecoveryDiag){0};
  first_config_fail_reg = 0xFFU;
  first_config_fail_status = HAL_OK;
  first_config_fail_state = first_config_fail_sr2 = 0;
  writes = reinitializations = gpio_changes = delayed_ms = 0;
  release_after = clock_edges = test_ipsr = test_primask = 0;
  scl_stuck = sda_stuck = busy_after_reset = 0;
  fail_at = 1;
  first_failure = HAL_BUSY;
  retry_failure = reinit_failure = HAL_OK;
}

static void verify_full_configuration(unsigned offset)
{
  static const uint8_t regs[] = {9, 8, 10, 12, 13, 4, 5, 6, 9};
  static const uint8_t values[] = {0x40, 0x10, 0x27, 0x1F, 0x1F, 0, 0, 0, 3};
  assert(writes == offset + sizeof(regs));
  assert(memcmp(written_regs + offset, regs, sizeof(regs)) == 0);
  assert(memcmp(written_values + offset, values, sizeof(values)) == 0);
}

int main(void)
{
  reset_case(); first_failure = HAL_OK;
  assert(MAX30102_Init() == HAL_OK);
  assert(max30102_recovery.attempts == 0 && gpio_changes == 0);
  verify_full_configuration(0);
  puts("PASS normal startup does not touch GPIO recovery");

  reset_case();
  assert(MAX30102_Init() == HAL_OK);
  assert(max30102_recovery.stage == MAX30102_RECOVERY_DONE);
  assert(max30102_recovery.attempts == 1 && max30102_recovery.clock_pulses == 0);
  assert(max30102_recovery.error_before == 0x20 && max30102_recovery.sr2_before == 2);
  assert(first_config_fail_reg == 9 && first_config_fail_status == HAL_BUSY);
  verify_full_configuration(1);
  puts("PASS idle-line BUSY recovers once and retains original failure");

  reset_case(); sda_stuck = 1; release_after = 3;
  assert(MAX30102_Init() == HAL_OK);
  assert(max30102_recovery.clock_pulses == 3);
  assert(max30102_recovery.sda_before == 0 && max30102_recovery.sda_after == 1);
  puts("PASS sensor releases SDA after three clocks");

  reset_case(); scl_stuck = 1;
  assert(MAX30102_Init() == HAL_TIMEOUT);
  assert(max30102_recovery.stage == MAX30102_RECOVERY_RELEASE_SCL);
  assert(writes == 1 && max30102_recovery.clock_pulses == 0);
  assert(test_gpiob.mode == GPIO_MODE_AF_OD);
  puts("PASS stuck SCL times out and restores pin mode without retrying writes");

  reset_case(); sda_stuck = 1;
  assert(MAX30102_Init() == HAL_TIMEOUT);
  assert(max30102_recovery.stage == MAX30102_RECOVERY_RELEASE_SDA);
  assert(max30102_recovery.clock_pulses == 9 && writes == 1);
  puts("PASS stuck SDA stops after nine clocks");

  reset_case(); first_failure = HAL_ERROR;
  assert(MAX30102_Init() == HAL_ERROR);
  assert(max30102_recovery.attempts == 0 && writes == 1 && gpio_changes == 0);
  puts("PASS missing-device NACK is not hidden by recovery");

  reset_case(); fail_at = 4; first_failure = HAL_TIMEOUT;
  assert(MAX30102_Init() == HAL_OK);
  assert(first_config_fail_reg == 12);
  verify_full_configuration(4);
  puts("PASS partial configuration failure restarts the entire sensor sequence");

  reset_case(); retry_failure = HAL_BUSY;
  assert(MAX30102_Init() == HAL_BUSY);
  assert(writes == 2 && reinitializations == 1 && max30102_recovery.attempts == 1);
  assert(max30102_recovery.retry_init_status == HAL_BUSY);
  puts("PASS second failure is returned with no retry loop");

  reset_case(); busy_after_reset = 1;
  assert(MAX30102_Init() == HAL_BUSY);
  assert(max30102_recovery.stage == MAX30102_RECOVERY_VERIFY_IDLE && writes == 1);
  puts("PASS persistent hardware BUSY is not reported as recovered");

  reset_case(); reinit_failure = HAL_ERROR;
  assert(MAX30102_Init() == HAL_ERROR);
  assert(max30102_recovery.stage == MAX30102_RECOVERY_REINIT && writes == 1);
  puts("PASS HAL reinitialization error is propagated");

  reset_case(); hi2c1.Lock = 1;
  assert(MAX30102_Init() == HAL_BUSY);
  assert(max30102_recovery.stage == MAX30102_RECOVERY_GUARD && gpio_changes == 0);
  reset_case(); test_primask = 1;
  assert(MAX30102_Init() == HAL_BUSY && gpio_changes == 0);
  reset_case(); test_ipsr = 1;
  assert(MAX30102_Init() == HAL_BUSY && gpio_changes == 0);
  puts("PASS locked handle and unsafe contexts do not drive the bus");
  return 0;
}
