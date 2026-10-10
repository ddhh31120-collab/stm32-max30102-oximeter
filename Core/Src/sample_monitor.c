#include "sample_monitor.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>



uint32_t check_count = 0;
uint32_t statistical_count_threshold = 0;
uint32_t statistical_time_threhold = 0;
uint32_t statistical_count_new = 0;
uint32_t statistical_time_new = 0;
uint32_t real_read_speed = 0;


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


uint8_t sample_active = 0;
uint8_t sample_count_flag = 0;


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




















