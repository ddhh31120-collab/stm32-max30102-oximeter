#include "ppg_filter.h"

#define PPG_DC_DIV               32L
#define PPG_FILTER_DIV            4L

int32_t red_dc = 0;                          /* 红光缓慢变化的直流基线 */
int32_t ir_dc = 0;                           /* 红外光缓慢变化的直流基线 */
int32_t red_ac = 0;                          /* 红光去掉基线后的脉搏变化，可正可负 */
int32_t ir_ac = 0;                           /* 红外光去掉基线后的脉搏变化，可正可负 */
uint8_t dc_filter_ready = 0U;                /* 0=尚未取得第一组样本，1=基线已初始化 */

int32_t red_filtered = 0;                    /* M3-C：平滑后的红光交流分量 */
int32_t ir_filtered = 0;                     /* M3-C：平滑后的红外光交流分量 */
uint8_t low_pass_ready = 0U;                 /* 0=尚无初始样本，1=低通滤波器已初始化 */


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

void PPG_ResetFilter(void)
{
  low_pass_ready = 0;
  dc_filter_ready = 0U;
}

