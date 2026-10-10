#ifndef SPO2_H
#define SPO2_H

#include <stdint.h>

extern uint8_t ef_num_ratio;
extern float ef_R;

extern uint8_t spo2_valid;
extern float spo2_est;


extern uint32_t win_flt_data_ptp_value_last[2];
extern uint32_t win_dc_data_ptp_value_last[2];
extern int32_t red_dc_avg;
extern int32_t ir_dc_avg;
extern uint8_t dc_stable;
extern int32_t sum_ir;
extern int32_t sum_red;

extern uint32_t win_data_bumber_now;



float relative_ratio_light(uint32_t *win_data_ptp_value_last,int32_t red_dc_avg,int32_t ir_dc_avg);
float SP02_estimation(float R,uint8_t ef_num_ratio);
uint8_t relative_change_dc(uint32_t *win_dc_data_ptp_value_last, int32_t red_dc_avg, int32_t ir_dc_avg);
void peak_to_peak_value_statistics(int32_t ir_filtered, int32_t red_filtered, int32_t ir_dc, int32_t red_dc);
void SPO2_ResetWindow(void);















#endif
