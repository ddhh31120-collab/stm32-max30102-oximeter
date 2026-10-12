/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "max30102.h"
#include "ppg_filter.h"
#include "heart_rate.h"
#include "spo2.h"
#include "sample_monitor.h"
#include "vofa_uart.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t red_raw;
  uint32_t ir_raw;
  int32_t red_filtered;
  int32_t ir_filtered;

  uint8_t beat_detected;
  uint32_t avg_bpm;
  uint8_t heart_rate_valid;

  float spo2_est;
  uint8_t spo2_valid;

  uint8_t sample_active;
}MeasurementFrame_t;

typedef enum
{
  RTOS_START_NO_ERROR = 0,
  RTOS_START_QUEUE_FAILED = 1,
  RTOS_START_TASK_A_FAILED = 2,
  RTOS_START_TASK_B_FAILED = 3,
  RTOS_START_KERNEL_INIT_FAILED = 4,
  RTOS_START_SCHEDULER_RETURNED = 5
}RTOSStartError_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for countTaskA */
osThreadId_t countTaskAHandle;
const osThreadAttr_t countTaskA_attributes = {
  .name = "countTaskA",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for countTaskB */
osThreadId_t countTaskBHandle;
const osThreadAttr_t countTaskB_attributes = {
  .name = "countTaskB",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};



/* USER CODE BEGIN PV */
uint8_t part_id = 0;                         /* Keil Watch: 预期 0x15 */
HAL_StatusTypeDef id_status = HAL_ERROR;     /* Keil Watch: 预期 HAL_OK */
uint8_t mode_reg = 0;                        /* Keil Watch: M2-A 完成后应为 0x03 */
HAL_StatusTypeDef mode_status = HAL_ERROR;   /* Keil Watch: 模式寄存器读取状态 */
HAL_StatusTypeDef config_status = HAL_ERROR; /* Keil Watch: 最近一次配置写入的状态 */

uint32_t red_raw = 0;                        /* 拼接后的红光原始值，最多 18 位 */
uint32_t ir_raw = 0;                         /* 拼接后的红外光原始值，最多 18 位 */

uint32_t sample_count = 0;                   /* 成功读取的样本组数，供 Watch 观察 */

HAL_StatusTypeDef fifo_status = HAL_ERROR;

volatile uint8_t test_pause_sample = 0;
volatile uint32_t Task_A_count = 0;
volatile uint32_t Task_B_count = 0;
volatile uint32_t stacksapce_A = 0;
volatile uint32_t stacksapce_B = 0;



osMessageQueueId_t measurementQueueHandle = NULL;

volatile osStatus_t Ret_status= osOK;
volatile uint32_t os_count = 0;  

volatile uint32_t queue_full_count = 0;

volatile uint8_t test_pause_receive = 0;
volatile uint8_t beat_queue_pending_watch = 0;

volatile RTOSStartError_t rtos_start_error = RTOS_START_NO_ERROR;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartCountTaskA(void *argument);
void StartCountTaskB(void *argument);

/* USER CODE BEGIN PFP */

void Result_invalid(uint8_t sample_active);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


  //两个任务单独计数

void Result_invalid(uint8_t sample_active)
{
  if(sample_active == 0)
  {
    heart_rate_valid = 0;//心率结果失效
    spo2_valid = 0;//血氧结果失效

    avg_bpm = 0;
     spo2_est = 0.0f;

    beat_detected = 0;
    HeartBeat_reset_status();//重置心跳采样状态
    PPG_ResetFilter();
    SPO2_ResetWindow();
  }
}

void Task_A()
{
  
  Task_A_count++;
}
void Task_B()
{
  
  Task_B_count++;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

   /* MAX30102_Init()
   * 每次把返回值交给 config_status；最后在 Watch 查看它是否为 HAL_OK。
   * 1. MODE_CONFIG <- 0x40：复位传感器，避免上一次调试留下旧设置。
   * 2. 复位后等待 10 ms；下面的 HAL_Delay 已经写好。
   * 3. FIFO_CONFIG <- 0x10：每次保留原始样本；满时允许覆盖旧数据。
   * 4. SPO2_CONFIG <- 0x27：ADC 量程 4096 nA、100 组/秒、18 位。
   * 5. LED1_PA、LED2_PA <- 0x1F：红光和红外光各约 6.2 mA。
   * 6. FIFO_WR_PTR、OVF_COUNTER、FIFO_RD_PTR <- 0x00：清空旧数据。
   * 7. MODE_CONFIG <- 0x03：最后启动 SpO2 模式（红光 + 红外光）。
    */
  config_status = MAX30102_Init();
  if(config_status != HAL_OK)
  {
    Error_Handler();
  }
  //ID 读取只需做一次：PART_ID 是固定值，不必每轮循环都访问 I2C。
  id_status = HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR_7BIT<<1,MAX30102_PART_ID_REG,I2C_MEMADD_SIZE_8BIT,&part_id,1,100);
  
  /* 模式配置也只回读一次：确认上面最后一笔写入让 MODE_CONFIG 变成 0x03。
   * mode_status 是 I2C 传输结果；mode_reg 是芯片返回的寄存器内容。 */
  mode_status = MAX30102_ReadBytes(MAX30102_REG_MODE_CONFIG, &mode_reg, 1U);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  if(osKernelGetState() != osKernelReady)
  {
    rtos_start_error = RTOS_START_KERNEL_INIT_FAILED;
    Error_Handler();
  }
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  measurementQueueHandle = osMessageQueueNew(4,sizeof(MeasurementFrame_t),NULL);
  if(measurementQueueHandle == NULL)
  {
    rtos_start_error = RTOS_START_QUEUE_FAILED;
    Error_Handler();
  }
  

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of countTaskA */
  countTaskAHandle = osThreadNew(StartCountTaskA, NULL, &countTaskA_attributes);

  /* creation of countTaskB */
  countTaskBHandle = osThreadNew(StartCountTaskB, NULL, &countTaskB_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if(countTaskAHandle == NULL)
  {
    rtos_start_error = RTOS_START_TASK_A_FAILED;
    Error_Handler();
  }
  if(countTaskBHandle == NULL)
  {
    rtos_start_error = RTOS_START_TASK_B_FAILED;
    Error_Handler();
  }
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   rtos_start_error = RTOS_START_SCHEDULER_RETURNED;
   Error_Handler();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCountTaskA */
/**
  * @brief  Function implementing the countTaskA thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCountTaskA */
void StartCountTaskA(void *argument)
{
  /* USER CODE BEGIN 5 */
  MeasurementFrame_t frame = {0};
  uint8_t beat_queue_pending = 0;
  /* Infinite loop */
  for(;;)
  {
    Task_A();
    beat_detected = 0U;
    /* TODO M2-B-1：两次指针读取都成功，且 wr_ptr != rd_ptr，才有未读样本。
     * 用一个 if 包住下面的 M2-B-2 和 M2-B-3；没数据时不要读 FIFO。 */
    uint8_t ready = 0;
    HAL_StatusTypeDef read_status = HAL_ERROR;
    read_status = MAX30102_DataReady(&ready);
    if(read_status == HAL_OK && ready == 1)
    {
        fifo_status = MAX30102_ReadSample(&red_raw, &ir_raw);
        if(fifo_status == HAL_OK && test_pause_sample == 0)
        {
          sample_count++;
          /* 先分离直流/交流，再把同一时刻的原始值和滤波值一起发送。 */
          PPG_RemoveDC(red_raw, ir_raw);
          PPG_LowPass(red_ac,ir_ac);
          peak_to_peak_value_statistics(ir_filtered,red_filtered,ir_dc,red_dc);
          /* 每取得一组新的滤波样本就检测一次；HAL_GetTick() 返回开机后的毫秒数。 */
          Heartbeat_DetectValley(ir_filtered, HAL_GetTick());
        }
        
        
    }
    
    sampling_rate_check(sample_count);//判断采样速率
    No_new_sample_No_old_sample(sample_count);//sample_active 是否有新样本
    Result_invalid(sample_active);//beat_detected = 0;
    //尚未收到样本或采样超时时，清除结果并重置算法历史
    if(sample_active == 0)beat_queue_pending = 0;
    else if(beat_detected == 1)beat_queue_pending = 1;
      frame.red_raw = red_raw;
      frame.ir_raw = ir_raw;
      frame.red_filtered = red_filtered;
      frame.ir_filtered = ir_filtered;
      frame.beat_detected = beat_queue_pending;
      frame.avg_bpm = avg_bpm;
      frame.heart_rate_valid = heart_rate_valid;
      frame.spo2_est = spo2_est;
      frame.spo2_valid = spo2_valid;
      frame.sample_active = sample_active;
      Ret_status = osMessageQueuePut(measurementQueueHandle,&frame,0,0);
    if(Ret_status == osOK){os_count++;beat_queue_pending = 0;}
    if(Ret_status == osErrorResource)queue_full_count++;

    beat_queue_pending_watch = beat_queue_pending;
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCountTaskB */
/**
* @brief Function implementing the countTaskB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCountTaskB */
void StartCountTaskB(void *argument)
{
  /* USER CODE BEGIN StartCountTaskB */
  MeasurementFrame_t receive_frame = {0};
  osStatus_t get_status = osOK;
  
  /* Infinite loop */
  for(;;)
  {
    if(test_pause_receive != 0)
    {
      osDelay(10);
      continue;
    }
    get_status = osMessageQueueGet(measurementQueueHandle,&receive_frame,NULL,osWaitForever);
    if(get_status == osOK)
    {
      Task_B();
      stacksapce_A = osThreadGetStackSpace(countTaskAHandle);
      stacksapce_B = osThreadGetStackSpace(countTaskBHandle);
      if(receive_frame.sample_active == 0)heart_pending = 0;
      UART_SendData(receive_frame.red_raw, receive_frame.ir_raw, receive_frame.red_filtered, receive_frame.ir_filtered, receive_frame.beat_detected, receive_frame.avg_bpm, receive_frame.heart_rate_valid, receive_frame.spo2_est,receive_frame.spo2_valid);
    }
    
  }
  /* USER CODE END StartCountTaskB */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
