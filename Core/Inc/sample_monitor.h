#ifndef SAMPLE_MONITOR_H
#define SAMPLE_MONITOR_H

#include <stdint.h>

extern uint32_t statistical_count_new;
extern uint32_t statistical_time_new;
extern uint32_t real_read_speed;

extern uint8_t sample_active;

void sampling_rate_check(uint32_t sample_count);
void No_new_sample_No_old_sample(uint32_t sample_count);



















#endif


