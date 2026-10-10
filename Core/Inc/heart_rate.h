#ifndef HEART_RATE_H
#define HEART_RATE_H
#include <stdint.h>

extern uint32_t interval_count[4];
extern uint16_t next_write_index;
extern uint16_t effective_number;
extern uint32_t avg_bpm;

extern int32_t ir_prev2;                        /* M4-A：波谷候选点前面的点 */
extern int32_t ir_prev1;                        /* M4-A：等待判断的中间点 */
extern uint8_t heartbeat_history_count;        /* 已保存的历史点数量，最大为 2 */
extern uint8_t beat_detected;                  /* 本次调用是否刚检测到心跳：1=是，0=否 */
extern uint32_t last_beat_ms;                  /* 上一次有效心跳的系统毫秒时间 */
extern uint32_t beat_count;                    /* 已检测到的心跳总数，供 Watch 验证 */
extern uint8_t beat_show_count;

extern uint32_t beat_interval_ms;
extern uint32_t inst_heart_rate_bpm;
extern uint8_t heart_rate_valid;
extern uint32_t beat_tobe;
extern uint32_t beat_deep_count;
extern uint32_t beat_time_count;
extern uint8_t detection_allow;


uint32_t avg_heart_rate(uint32_t beat_interval_ms);
void Heartbeat_DetectValley(int32_t ir_sample, uint32_t now_ms);
void HeartBeat_reset_status(void);





#endif





