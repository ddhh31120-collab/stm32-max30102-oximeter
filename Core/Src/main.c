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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  /* snprintf：把整数格式化为串口可发送的文本 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

/* M3-B：直流基线每次只向当前原始值靠近 1/32。
 * 除数越大，基线变化越慢；除数太小会把脉搏波也当成基线一起减掉。
 * 当前采样率为 100 Hz，32 是便于整数运算的起始参数。 */
#define PPG_DC_DIV               32L

/* M3-C：交流分量的一阶低通滤波系数。
 * 每次让滤波结果向新输入靠近差值的 1/4，用来减小细小的高频毛刺，
 * 同时保留变化速度较慢的脉搏波。 */
#define PPG_FILTER_DIV            4L

/* M4-A：两次有效心跳之间允许的最短时间。
 * 300 ms 对应 200 BPM。检测到一次波谷后，在这段时间内忽略新的小波谷，
 * 可以避免同一个脉搏附近的细小抖动被重复计数。 */
#define HEARTBEAT_REFRACTORY_MS  300U

/* M4-A：候选波谷必须低于这个值。
 * 当前去直流后的红外光波谷约为负数，因此先用 -100 作为实验起点；
 * 它不是芯片手册规定值，后面要结合实际波形调整。 */
#define HEARTBEAT_VALLEY_LEVEL  (-100L)
#define HEARTBEAT_RELEASE_LEVEL  (100L)

//平均光强计数门限
//无手指约 687～689 稳定放指约 127028～128922 手指距离传感器1厘米左右是25000左右
#define FINGER_DISTANCE_THRESHOLD (25000L)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t part_id = 0;                         /* Keil Watch: 预期 0x15 */
HAL_StatusTypeDef id_status = HAL_ERROR;     /* Keil Watch: 预期 HAL_OK */
uint8_t mode_reg = 0;                        /* Keil Watch: M2-A 完成后应为 0x03 */
HAL_StatusTypeDef mode_status = HAL_ERROR;   /* Keil Watch: 模式寄存器读取状态 */
HAL_StatusTypeDef config_status = HAL_ERROR; /* Keil Watch: 最近一次配置写入的状态 */
uint8_t first_config_fail_reg = 0xFFU;       /* 0xFF = 九次配置写入均未报告失败 */
HAL_StatusTypeDef first_config_fail_status = HAL_OK;
uint32_t first_config_fail_state = 0;        /* 第一次失败时 HAL I2C 句柄的状态 */
uint32_t first_config_fail_sr2 = 0;          /* 第一次失败时 I2C1 状态寄存器 SR2 */
uint8_t fifo_wr_ptr = 0;                     /* 传感器下一次要写的 FIFO 位置 */
uint8_t fifo_rd_ptr = 0;                     /* MCU 下一次要读的 FIFO 位置 */
uint8_t fifo_bytes[6] = {0};                 /* 一组样本：红光 3 字节、红外光 3 字节 */
uint32_t red_raw = 0;                        /* 拼接后的红光原始值，最多 18 位 */
uint32_t ir_raw = 0;                         /* 拼接后的红外光原始值，最多 18 位 */
int32_t red_dc = 0;                          /* 红光缓慢变化的直流基线 */
int32_t ir_dc = 0;                           /* 红外光缓慢变化的直流基线 */
int32_t red_ac = 0;                          /* 红光去掉基线后的脉搏变化，可正可负 */
int32_t ir_ac = 0;                           /* 红外光去掉基线后的脉搏变化，可正可负 */
uint8_t dc_filter_ready = 0U;                /* 0=尚未取得第一组样本，1=基线已初始化 */
uint32_t sample_count = 0;                   /* 成功读取的样本组数，供 Watch 观察 */
HAL_StatusTypeDef wr_status = HAL_ERROR;
HAL_StatusTypeDef rd_status = HAL_ERROR;
HAL_StatusTypeDef fifo_status = HAL_ERROR;
HAL_StatusTypeDef uart_status = HAL_ERROR;   /* 最近一次串口发送状态，供 Watch 观察 */

int32_t red_filtered = 0;                    /* M3-C：平滑后的红光交流分量 */
int32_t ir_filtered = 0;                     /* M3-C：平滑后的红外光交流分量 */
uint8_t low_pass_ready = 0U;                 /* 0=尚无初始样本，1=低通滤波器已初始化 */
int32_t ir_prev2 = 0;                        /* M4-A：波谷候选点前面的点 */
int32_t ir_prev1 = 0;                        /* M4-A：等待判断的中间点 */
uint8_t heartbeat_history_count = 0U;        /* 已保存的历史点数量，最大为 2 */
uint8_t beat_detected = 0U;                  /* 本次调用是否刚检测到心跳：1=是，0=否 */
uint32_t last_beat_ms = 0U;                  /* 上一次有效心跳的系统毫秒时间 */
uint32_t beat_count = 0U;                    /* 已检测到的心跳总数，供 Watch 验证 */
uint8_t beat_show_count = 0U;

uint32_t beat_interval_ms = 0U;
uint32_t inst_heart_rate_bpm = 0U;
uint8_t heart_rate_valid = 0U;

uint32_t beat_tobe = 0;
uint32_t beat_deep_count = 0;
uint32_t beat_time_count = 0;
uint8_t detection_allow = 1U;

uint32_t interval_count[4] = {0};
uint16_t next_write_index = 0;
uint16_t effective_number = 0;
uint32_t avg_bpm = 0;

//win_data_max_now[2]以及以下两个数组，数组[0]代表红外光，数组[1]代表红光
uint32_t win_data_bumber_now = 0;
int32_t win_flt_data_max_now[2] = {0,0};
int32_t win_flt_data_min_now[2] = {0,0};
int32_t win_dc_data_max_now[2] = {0,0};
int32_t win_dc_data_min_now[2] = {0,0};
uint32_t win_flt_data_ptp_value_last[2] = {0,0};
uint32_t win_dc_data_ptp_value_last[2] = {0,0};

int32_t red_dc_avg = 0;
int32_t ir_dc_avg = 0;
int32_t sum_ir = 0;
int32_t sum_red = 0;

uint8_t ef_num_ratio = 0;
float ef_R = 0;

uint8_t spo2_valid = 0;
float spo2_est = 0;

uint8_t dc_stable = 0;

uint32_t check_count = 0;
uint32_t statistical_count_threshold = 0;
uint32_t statistical_time_threhold = 0;
uint32_t statistical_count_new = 0;
uint32_t statistical_time_new = 0;
uint32_t real_read_speed = 0;

uint8_t sample_processed = 5;
uint8_t heart_pending = 0;
uint8_t sample_active = 0;
uint8_t sample_count_flag = 0;

volatile uint8_t test_pause_sample = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef MAX30102_ReadBytes(uint8_t reg, uint8_t *data, uint16_t size);
void PPG_RemoveDC(uint32_t red, uint32_t ir);
void PPG_LowPass(int32_t red_input, int32_t ir_input);
void Heartbeat_DetectValley(int32_t ir_sample, uint32_t now_ms);
void UART_SendData(uint32_t red, uint32_t ir, int32_t red_pulse, int32_t ir_pulse, uint8_t beat_detected, uint32_t heart_rate_bpm, uint8_t heart_rate_valid, float SPO2_est, uint8_t spo2_valid);
uint32_t avg_heart_rate(uint32_t beat_interval_ms);
void peak_to_peak_value_statistics(int32_t ir_filtered, int32_t red_filtered, int32_t ir_dc, int32_t red_dc);
float relative_ratio_light(uint32_t *win_data_ptp_value_last,int32_t red_dc_avg,int32_t ir_dc_avg);
uint8_t relative_change_dc(uint32_t *win_dc_data_ptp_value_last, int32_t red_dc_avg, int32_t ir_dc_avg);
void sampling_rate_check(uint32_t sample_count);
void No_new_sample_No_old_sample(uint32_t sample_count);
void Result_invalid(uint8_t sample_active);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 向 MAX30102 的一个 8 位寄存器写入一个字节。
 * 为什么要用 HAL_I2C_Mem_Write：传感器内部有很多寄存器。这个函数会先发送
 * 寄存器地址 reg，再发送要写入的 value；我们不必手写 I2C 起始/停止信号。
 * 返回 HAL_OK 表示传输成功，其余状态表示写入没有正常完成。 */
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

/* M3-B：从原始值中分离缓慢变化的直流基线和脉搏交流分量。
 * red、ir 是同一时刻的两个 18 位无符号 ADC 值。
 * red_dc、ir_dc 用低通方式跟随手指压力、组织吸收和环境光造成的慢变化；
 * red_ac、ir_ac = 原始值 - 基线，因此必须用有符号 int32_t 保存。 */
void PPG_RemoveDC(uint32_t red, uint32_t ir)
{
  /* TODO M3-B-1：第一组样本只用于初始化基线，避免启动时产生巨大尖峰。
   * 当 dc_filter_ready == 0U 时：
   *   red_dc、ir_dc 分别设为当前 red、ir（转换成 int32_t）；
   *   red_ac、ir_ac 清零；dc_filter_ready 置 1U；然后 return。
   * 为什么要 return：第一组数据只确定“起点”，还没有足够历史可分离脉搏。 */
  if(dc_filter_ready == 0U)
  {
    red_dc = (int32_t)red;
    ir_dc = (int32_t)ir;
    red_ac = 0;ir_ac = 0;
    dc_filter_ready = 1;
    return; 
  }

  /* TODO M3-B-2：分别处理红光和红外光，每路两行。
   * 第一步更新基线：
   *   dc += ((int32_t)raw - dc) / PPG_DC_DIV;
   * 含义：计算“当前值与旧基线的距离”，每次只走这段距离的 1/32。
   * 第二步得到脉搏分量：
   *   ac = (int32_t)raw - dc;
   * 顺序不能反：先更新本次基线，再用本次原始值减去它。 */
  red_dc += ((int32_t)red - red_dc) / PPG_DC_DIV;
  ir_dc += ((int32_t)ir - ir_dc) / PPG_DC_DIV;
  red_ac = (int32_t)red - red_dc;
  ir_ac = (int32_t)ir - ir_dc;
}

/* 串口输出四列：原始红光、原始红外、去直流红光、去直流红外。
 * M2-C 已经练过文本格式化与串口发送，因此这里直接扩展为四通道。
 * %lu 对应无符号原始值；%ld 对应可能为负数的去直流结果。 */
/* M3-C：平滑 PPG_RemoveDC() 得到的交流分量。
 * red_input、ir_input 使用 int32_t，是因为交流波形可能高于或低于基线。
 * red_filtered、ir_filtered 是全局状态，会在相邻两次调用之间保留旧结果。 */
void PPG_LowPass(int32_t red_input, int32_t ir_input)
{
  /* 第一组样本直接作为滤波器起点。如果从 0 开始逐渐靠近输入，启动时会产生
   * 一个并非来自心跳的过渡过程。 */
  if (low_pass_ready == 0U)
  {
    red_filtered = red_input;
    ir_filtered = ir_input;
    low_pass_ready = 1U;
    return;
  }

  /* TODO M3-C-1：更新 red_filtered。
   * 公式：filtered += (input - filtered) / PPG_FILTER_DIV;
   * 含义：求新输入与旧结果的距离，本次只走这段距离的 1/4。 */
   red_filtered += (red_input - red_filtered) / PPG_FILTER_DIV;
  /* TODO M3-C-2：用相同公式更新 ir_filtered。 */
  ir_filtered += (ir_input - ir_filtered) / PPG_FILTER_DIV;
}

/*SPO2估算函数 
目的：估算SPO2
参数 R：红光相对波动与红外相对波动的比值 无单位 要求大于0
参数 ef_num_ratio：本窗口的R的有效标志，有效比值标志位 1=有效 0=无效
返回值：估算的SPO2值，范围0~100 返回的SPO2是百分数，所以才要返回0-100
状态：更新全局spo2_valid 1为通过本轮检查 0为未通过
状态判定依据：R无效时候不带入公式，且结果在0-100范围内


*/
float SP02_estimation(float R,uint8_t ef_num_ratio)
{
  float spo2_est = 0;
  
  if(ef_num_ratio == 1 && R > 0)//R=0 代入公式也会得到 94.845，因此必须先排除无效输入，避免把无效窗口转换成看起来正常的估算结果
  {
    spo2_est = -45.060 * R * R + 30.354 * R + 94.845;//公式来源：https://www.maximintegrated.com/en/design/technical-documents/app-notes/5/5817.html

    if(spo2_est < 0 || spo2_est > 100)
    {
      spo2_valid = 0;
      spo2_est = 0;
    }
    else spo2_valid = 1;
  }
  else
  {
    spo2_valid = 0;
    spo2_est = 0;
  }
  return spo2_est;
}

float relative_ratio_light(uint32_t *win_data_ptp_value_last,int32_t red_dc_avg,int32_t ir_dc_avg)
{
  float red_ratio = 0;
  float ir_ratio = 0;
  float R = 0;
  if(win_data_ptp_value_last[1] > 0 && win_data_ptp_value_last[0] > 0 && red_dc_avg > 0 && ir_dc_avg > 0)
  {
    red_ratio = (float)win_data_ptp_value_last[1]  / (float)red_dc_avg;
    ir_ratio = (float)win_data_ptp_value_last[0]  / (float)ir_dc_avg;
    R = red_ratio / ir_ratio;
    ef_num_ratio = 1;
  }
  //if(win_data_ptp_value_last[1] == 0 || win_data_ptp_value_last[0] == 0 || red_dc_avg == 0 || ir_dc_avg == 0)
  else ef_num_ratio = 0;
  return R;
}

//程序能完成除法，但是并不能说明输入的波动一定来自我们想测量的脉搏，所以要新增加
//一个检查，去看看四秒的窗口里，DC这个基线移动了多少
//返回值：1 == 两路基线都通过稳定性检查   0 == 输入无效或任意一门变化超过门限
//
uint8_t relative_change_dc(uint32_t *win_dc_data_ptp_value_last, int32_t red_dc_avg, int32_t ir_dc_avg)
{
  if(ir_dc_avg > 0 && red_dc_avg > 0)
  {
    float ir_relative_change_dc = (float)win_dc_data_ptp_value_last[0] / (float)ir_dc_avg;
    float red_relative_change_dc = (float)win_dc_data_ptp_value_last[1] / (float)red_dc_avg;
    if(ir_relative_change_dc <= 0.05f && red_relative_change_dc <= 0.05f)return 1;
    else return 0;
  }
  else return 0;
  
}
void peak_to_peak_value_statistics(int32_t ir_filtered, int32_t red_filtered, int32_t ir_dc, int32_t red_dc)
{
  int32_t red_dc_temp = 0;
  int32_t ir_dc_temp = 0;
  
  if(win_data_bumber_now == 0)
  {
    win_flt_data_max_now[0] = ir_filtered;
    win_flt_data_min_now[0] = ir_filtered;

    win_flt_data_max_now[1] = red_filtered;
    win_flt_data_min_now[1] = red_filtered;
    //最大值减去最小值就是AC,filtered是滤波后的波形，波形的最大值减去最小值
    //就可以近似地求ac
    //求R用的AC是个幅度，而不是某一次采样地red_ac或者ir_ac。我们这里求幅度的目的
    //就是求R，然后用R去估算spo2


    win_dc_data_max_now[0] = ir_dc;
    win_dc_data_min_now[0] = ir_dc;

    win_dc_data_max_now[1] = red_dc;
    win_dc_data_min_now[1] = red_dc;
    ir_dc_temp = ir_dc;
    red_dc_temp = red_dc;
    
  }
  else
  {
    if(ir_filtered > win_flt_data_max_now[0]) win_flt_data_max_now[0] = ir_filtered;
    if(ir_filtered < win_flt_data_min_now[0]) win_flt_data_min_now[0] = ir_filtered;
    if(red_filtered > win_flt_data_max_now[1]) win_flt_data_max_now[1] = red_filtered;
    if(red_filtered < win_flt_data_min_now[1]) win_flt_data_min_now[1] = red_filtered;
    if(ir_dc > win_dc_data_max_now[0])win_dc_data_max_now[0] = ir_dc;
    if(ir_dc < win_dc_data_min_now[0])win_dc_data_min_now[0] = ir_dc;
    if(red_dc > win_dc_data_max_now[1])win_dc_data_max_now[1] = red_dc;
    if(red_dc < win_dc_data_min_now[1])win_dc_data_min_now[1] = red_dc;
    ir_dc_temp = ir_dc;
    red_dc_temp = red_dc;
  }
  sum_ir += ir_dc_temp;
  sum_red += red_dc_temp;
  win_data_bumber_now++;
  if(win_data_bumber_now == 400)
  {
    win_flt_data_ptp_value_last[0] = win_flt_data_max_now[0] - win_flt_data_min_now[0];
    win_flt_data_ptp_value_last[1] = win_flt_data_max_now[1] - win_flt_data_min_now[1];
    win_dc_data_ptp_value_last[0] = win_dc_data_max_now[0] - win_dc_data_min_now[0];
    win_dc_data_ptp_value_last[1] = win_dc_data_max_now[1] - win_dc_data_min_now[1];

    ir_dc_avg = sum_ir / win_data_bumber_now;
    red_dc_avg = sum_red / win_data_bumber_now;
    dc_stable = relative_change_dc(win_dc_data_ptp_value_last,red_dc_avg,ir_dc_avg);
    sum_ir = 0;
    sum_red = 0;
    if(ir_dc_avg > FINGER_DISTANCE_THRESHOLD)
    {
      
      if(dc_stable)
      {
        ef_R = relative_ratio_light(win_flt_data_ptp_value_last, red_dc_avg, ir_dc_avg);
      }
      else
      {
        ef_R = 0;
        ef_num_ratio = 0;
      }
      
    }
    else
    {
      ef_R = 0;
      ef_num_ratio = 0;
    }
    spo2_est = SP02_estimation(ef_R, ef_num_ratio);
    win_data_bumber_now = 0;
  }
}

uint32_t avg_heart_rate(uint32_t beat_interval_ms)
{
  uint32_t avg = 0;
  interval_count[next_write_index] = beat_interval_ms;//保存新间隔
  next_write_index++;//更新下一步写入位置
  if(next_write_index >= 4) next_write_index = 0;//循环写入
  if(effective_number < 4)effective_number++;//有效数量++
  uint32_t sum = 0;
  for(int i = 0; i < 4; i++)
  {
    sum += interval_count[i];//遍历求和
  }
  avg = sum / effective_number;
  return 60000U / avg;
}


/* M4-A：利用连续三个红外光滤波值寻找局部波谷。
 * ir_sample：本次最新的红外光滤波值。
 * now_ms：调用者通过 HAL_GetTick() 取得的当前时间，单位为 ms。
 *
 * 当已有 A=ir_prev2、B=ir_prev1，并收到新点 C=ir_sample 时，
 * 如果 B 同时小于 A 和 C，那么 B 是一个局部波谷候选点。
 * 函数没有返回值；结果写入 beat_detected，累计次数写入 beat_count。 */
void Heartbeat_DetectValley(int32_t ir_sample, uint32_t now_ms)
{
  /* beat_detected 只表示“本次调用刚刚检测到”。如果本次没有检测到，
   * 必须先清零，否则一次心跳会在后续很多组数据中一直保持为 1。 */
  beat_detected = 0U;
  if (heartbeat_history_count == 0U)
  {
    ir_prev2 = ir_sample;//现在的采样数据放给“前两个采样点”
    heartbeat_history_count = 1U;
    return;
  }

  if (heartbeat_history_count == 1U)
  {
    ir_prev1 = ir_sample;//现在的采样数据放给上一个采样点
    heartbeat_history_count = 2U;//历史采样次数从此变为两次，加上正在进行的这一次，数据足够
    return;
  }
  /* 判断局部波谷需要三个点。前两次调用只负责收集历史数据。 */

  //要想收集到波谷，就要让 ir_prev1 处于中间位置，ir_prev2 处于前一个位置，ir_sample 处于后一个位置。
  //即ir_prev1最小

  /* TODO M4-A-1：用一个 if 同时判断以下三个条件：
   * 1. ir_prev1 < ir_prev2，并且 ir_prev1 <= ir_sample：中间点是局部波谷；
   * 2. ir_prev1 < HEARTBEAT_VALLEY_LEVEL：波谷足够深，不把零点附近噪声当心跳；
   * 3. (uint32_t)(now_ms - last_beat_ms) >= HEARTBEAT_REFRACTORY_MS：
   *    距离上次心跳至少 300 ms。使用无符号减法还能正确处理计时器回绕。
   *
   * 条件成立时，在 if 内完成三件事：
   * beat_detected 置 1U；beat_count 加 1U；last_beat_ms 更新为 now_ms。 */

  /* 无论本次是否检测到波谷，都要把窗口向前移动一格：
   * 原来的 B 成为下一轮的 A，本次新点 C 成为下一轮的 B。 */
   //检测到了有效波谷:
   if(detection_allow == 1U)
   {
    if(ir_prev1 < ir_prev2 && ir_prev1 <= ir_sample && ir_prev1 < HEARTBEAT_VALLEY_LEVEL && (uint32_t)(now_ms - last_beat_ms) >= HEARTBEAT_REFRACTORY_MS)
    {
      if(beat_count == 0)
      {
        last_beat_ms = now_ms;
      }
      else
      {
        beat_interval_ms = now_ms - last_beat_ms;
        if(beat_interval_ms > 0)
        {
          inst_heart_rate_bpm = (60000U / beat_interval_ms);
          heart_rate_valid = 1U;
          avg_bpm = avg_heart_rate(beat_interval_ms);
        }
        else heart_rate_valid = 0;
        last_beat_ms = now_ms;
      }
      beat_detected = 1u;
      beat_count++;
      detection_allow = 0U;
    }
   }
   else
   {
    if(ir_sample > HEARTBEAT_RELEASE_LEVEL)
    {
      detection_allow = 1U;
    }
   }
    if(now_ms - last_beat_ms > 3000U)
    {
      heart_rate_valid = 0;
      for(int i = 0; i < 4; i++)
      {
        interval_count[i] = 0;
      }
      next_write_index = 0;
      effective_number = 0;
      avg_bpm = 0;
      beat_count = 0;
    }
  ir_prev2 = ir_prev1;
  ir_prev1 = ir_sample;
}
//检查实际采样速率
void sampling_rate_check(uint32_t sample_count)
{
  uint32_t time = HAL_GetTick();

  if(check_count == 0)
  {
    statistical_count_threshold = sample_count;
    statistical_time_threhold = time;
    check_count++;
  }
  if(check_count > 0 && (time - statistical_time_threhold) >= 1000)
  {
    statistical_count_new = sample_count - statistical_count_threshold;
    statistical_time_new = time - statistical_time_threhold;
    real_read_speed = statistical_count_new * 1000 / statistical_time_new;
    statistical_count_threshold = sample_count;
    statistical_time_threhold = time;
  }
  
}

void No_new_sample_No_old_sample(uint32_t sample_count)
{

  static uint32_t last_sample = 0;
  static uint32_t time_last = 0;
  uint32_t time_new = 0;
  uint32_t time_count = 0;

  if(sample_count_flag == 0)
  {
    if(sample_count == 0)
    {
      sample_active = 0;
    }
    else if(sample_count > 0)
    {
      sample_active = 1;
    }
    last_sample = sample_count;
    sample_count_flag++;
    time_last = HAL_GetTick();
  }
  else
  {
    time_new = HAL_GetTick();
    if(last_sample == sample_count)
    {
      time_count = time_new - time_last;
      if(time_count >= 1000)
      {
        sample_active = 0;
      }
    }
    else
    {
      sample_active = 1;
      last_sample = sample_count;
      time_last = HAL_GetTick();
    }
  }
}

void Result_invalid(uint8_t sample_active)
{
  if(sample_active == 0)
  {
    heart_rate_valid = 0;//心率结果失效
    spo2_valid = 0;//血氧结果失效

    avg_bpm = 0;
     spo2_est = 0.0f;

    beat_detected = 0;
     heart_pending = 0;
  }
}

void HeartBeat_reset_status()
{
  
}
void UART_SendData(uint32_t red, uint32_t ir, int32_t red_pulse, int32_t ir_pulse, uint8_t beat_detected, uint32_t heart_rate_bpm, uint8_t heart_rate_valid, float spo2_est, uint8_t spo2_valid)
{
  static uint8_t send_countdown = 5;  // 距离下次发送还需要处理的采样组数
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
  send_countdown--;
  uint32_t heart_rate_to_sent = heart_rate_valid ? heart_rate_bpm : 0U;
  if(send_countdown == 0)
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
    send_countdown = 5;
  }
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
  /* TODO M2-A：按下面的顺序调用 MAX30102_WriteReg(reg, value)。
   * 每次把返回值交给 config_status；最后在 Watch 查看它是否为 HAL_OK。
   * 1. MODE_CONFIG <- 0x40：复位传感器，避免上一次调试留下旧设置。
   * 2. 复位后等待 10 ms；下面的 HAL_Delay 已经写好。
   * 3. FIFO_CONFIG <- 0x10：每次保留原始样本；满时允许覆盖旧数据。
   * 4. SPO2_CONFIG <- 0x27：ADC 量程 4096 nA、100 组/秒、18 位。
   * 5. LED1_PA、LED2_PA <- 0x1F：红光和红外光各约 6.2 mA。
   * 6. FIFO_WR_PTR、OVF_COUNTER、FIFO_RD_PTR <- 0x00：清空旧数据。
   * 7. MODE_CONFIG <- 0x03：最后启动 SpO2 模式（红光 + 红外光）。
   * 只需写调用语句；上面的寄存器常量和函数骨架已经准备好。 */
  /* TODO M2-A-1：复位调用写在这里 */
  config_status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG,0x40);

  HAL_Delay(10); /* 等传感器完成复位；它不会自动执行复位，仍需完成上面的 TODO。 */
  /* TODO M2-A-2：其余八次寄存器写入写在这里 */
  config_status = MAX30102_WriteReg(MAX30102_REG_FIFO_CONFIG,0x10);
  config_status = MAX30102_WriteReg(MAX30102_REG_SPO2_CONFIG,0x27);
  config_status = MAX30102_WriteReg(MAX30102_REG_LED1_PA,0x1F);
  config_status = MAX30102_WriteReg(MAX30102_REG_LED2_PA,0x1F);
  config_status = MAX30102_WriteReg(MAX30102_REG_FIFO_WR_PTR,0x00);
  config_status = MAX30102_WriteReg(MAX30102_REG_OVF_COUNTER,0x00);
  config_status = MAX30102_WriteReg(MAX30102_REG_FIFO_RD_PTR,0x00);
  config_status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG,0x03);

  /* M1 的 ID 读取只需做一次：PART_ID 是固定值，不必每轮循环都访问 I2C。
   * 第一个参数选总线；第二个参数选设备；第三个参数选 PART_ID 寄存器；
   * 后面依次说明寄存器地址宽度、结果存放地址、字节数和超时。 */
  id_status = HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR_7BIT<<1,MAX30102_PART_ID_REG,I2C_MEMADD_SIZE_8BIT,&part_id,1,100);
  /* 模式配置也只回读一次：确认上面最后一笔写入让 MODE_CONFIG 变成 0x03。
   * mode_status 是 I2C 传输结果；mode_reg 是芯片返回的寄存器内容。 */
  mode_status = MAX30102_ReadBytes(MAX30102_REG_MODE_CONFIG, &mode_reg, 1U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* M2-B：每轮查看 FIFO 的写/读指针。指针只占 5 位，范围 0~31；
     * 两次读取的 size 都是 1，因为每个指针寄存器只有一个字节。 */
    wr_status = MAX30102_ReadBytes(MAX30102_REG_FIFO_WR_PTR, &fifo_wr_ptr, 1U);
    rd_status = MAX30102_ReadBytes(MAX30102_REG_FIFO_RD_PTR, &fifo_rd_ptr, 1U);
    /* TODO M2-B-1：两次指针读取都成功，且 wr_ptr != rd_ptr，才有未读样本。
     * 用一个 if 包住下面的 M2-B-2 和 M2-B-3；没数据时不要读 FIFO。 */
    if(wr_status == HAL_OK && rd_status == HAL_OK && fifo_wr_ptr != fifo_rd_ptr)
    {

      fifo_status = MAX30102_ReadBytes(MAX30102_REG_FIFO_DATA,fifo_bytes,6u);
      if(fifo_status == HAL_OK && test_pause_sample == 0)
      {
        red_raw = ((uint32_t)fifo_bytes[0] << 16)
                 |((uint32_t)fifo_bytes[1] << 8)
                 |((uint32_t)fifo_bytes[2]);
        ir_raw = ((uint32_t)fifo_bytes[3] << 16)
                 |((uint32_t)fifo_bytes[4] << 8)
                 |((uint32_t)fifo_bytes[5]);
        red_raw &= 0x3FFFFU;
        ir_raw &= 0x3FFFFU;
        sample_count++;
        /* 先分离直流/交流，再把同一时刻的原始值和滤波值一起发送。 */
        PPG_RemoveDC(red_raw, ir_raw);
        PPG_LowPass(red_ac,ir_ac);
        peak_to_peak_value_statistics(ir_filtered,red_filtered,ir_dc,red_dc);
        /* 每取得一组新的滤波样本就检测一次；HAL_GetTick() 返回开机后的毫秒数。 */
        Heartbeat_DetectValley(ir_filtered, HAL_GetTick());
        UART_SendData(red_raw, ir_raw, red_filtered, ir_filtered, beat_detected, avg_bpm, heart_rate_valid, spo2_est,spo2_valid);
      }
    }
    sampling_rate_check(sample_count);
    No_new_sample_No_old_sample(sample_count);
    Result_invalid(sample_active);
    /* TODO M2-B-2：从 FIFO_DATA 一次读 6 字节到 fifo_bytes，
     * 返回状态存入 fifo_status。只有 fifo_status == HAL_OK 才处理数据。 */
    
    /* TODO M2-B-3：按大端顺序拼接：前 3 字节是 red_raw，后 3 字节是 ir_raw。
     * 每个结果只保留低 18 位（掩码 0x3FFFF）；成功后 sample_count++。
     * 提示：先把每个 uint8_t 转为 uint32_t，再左移 16 位或 8 位。 */

    HAL_Delay(1); /* 传感器每 10 ms 产一组；每 5 ms 查看一次以便及时取走。 */
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
