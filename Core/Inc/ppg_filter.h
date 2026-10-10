#ifndef PPG_FILTER
#define PPG_FILTER

#include <stdint.h>


extern int32_t red_dc;                          
extern int32_t ir_dc;                          
extern int32_t red_ac;                          
extern int32_t ir_ac;                           
extern uint8_t dc_filter_ready;

extern int32_t red_filtered;
extern int32_t ir_filtered;
extern uint8_t low_pass_ready;

void PPG_RemoveDC(uint32_t red, uint32_t ir);
void PPG_LowPass(int32_t red_input, int32_t ir_input);
void PPG_ResetFilter(void);










#endif

