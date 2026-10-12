#ifndef TEST_STM32F1XX_HAL_H
#define TEST_STM32F1XX_HAL_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t CR1, SR2; } I2C_TypeDef;
typedef struct { uint32_t ODR, mode; } GPIO_TypeDef;
typedef struct {
  I2C_TypeDef *Instance;
  uint32_t State, Lock, ErrorCode;
} I2C_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
extern I2C_TypeDef test_i2c1;
extern GPIO_TypeDef test_gpiob;
extern uint32_t test_ipsr, test_primask;
#define I2C1 (&test_i2c1)
#define GPIOB (&test_gpiob)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_MODE_AF_OD 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 3U
#define HAL_UNLOCKED 0U
#define HAL_I2C_STATE_READY 0x20U
#define HAL_I2C_ERROR_TIMEOUT 0x20U
#define I2C_FLAG_BUSY 2U
#define I2C_MEMADD_SIZE_8BIT 1U
#define RESET 0U
#define __get_IPSR() test_ipsr
#define __get_PRIMASK() test_primask
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_I2C_DISABLE(h) ((h)->Instance->CR1 &= ~1U)
#define __HAL_I2C_GET_FLAG(h, flag) ((h)->Instance->SR2 & (flag))
void HAL_Delay(uint32_t ms);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout);
#endif
