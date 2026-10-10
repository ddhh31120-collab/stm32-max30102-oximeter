#include "heart_rate.h"


#define HEARTBEAT_REFRACTORY_MS  300U
#define HEARTBEAT_VALLEY_LEVEL  (-100L)
#define HEARTBEAT_RELEASE_LEVEL  (100L)

uint32_t interval_count[4] = {0};
uint16_t next_write_index = 0;
uint16_t effective_number = 0;
uint32_t avg_bpm = 0;

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

void Heartbeat_DetectValley(int32_t ir_sample, uint32_t now_ms)
{
  /* beat_detected 只表示“本次调用刚刚检测到”。如果本次没有检测到，
   * 必须先清零，否则一次心跳会在后续很多组数据中一直保持为 1。 */



   /* 利用连续三个红外光滤波值寻找局部波谷。
 * ir_sample：本次最新的红外光滤波值。
 * now_ms：调用者通过 HAL_GetTick() 取得的当前时间，单位为 ms。
 *
 * 当已有 A=ir_prev2、B=ir_prev1，并收到新点 C=ir_sample 时，
 * 如果 B 同时小于 A 和 C，那么 B 是一个局部波谷候选点。
 * 函数没有返回值；结果写入 beat_detected，累计次数写入 beat_count。 */
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

void HeartBeat_reset_status(void)
{
  heartbeat_history_count = 0;
  detection_allow = 1;
  beat_interval_ms = 0;
  last_beat_ms = 0;
  inst_heart_rate_bpm = 0;
  ir_prev1 = 0;
  ir_prev2 = 0;
  for(int i = 0; i < 4; i++)
  {
    interval_count[i] = 0;
  }
  next_write_index = 0;
  effective_number = 0;
  avg_bpm = 0;
  beat_count = 0;
}






