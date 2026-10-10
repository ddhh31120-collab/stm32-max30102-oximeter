#include "spo2.h"
//平均光强计数门限
//无手指约 687～689 稳定放指约 127028～128922 手指距离传感器1厘米左右是25000左右
#define FINGER_DISTANCE_THRESHOLD (25000L)

uint8_t ef_num_ratio = 0;
float ef_R = 0;



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


/*SPO2估算函数 
目的：估算SPO2
参数 R：红光相对波动与红外相对波动的比值 无单位 要求大于0
参数 ef_num_ratio：本窗口的R的有效标志，有效比值标志位 1=有效 0=无效
返回值：估算的SPO2值，范围0~100 返回的SPO2是百分数，所以才要返回0-100
状态：更新全局spo2_valid 1为通过本轮检查 0为未通过
状态判定依据：R无效时候不带入公式，且结果在0-100范围内
*/
uint8_t spo2_valid = 0;
float spo2_est = 0;

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

//win_data_max_now[2]以及以下两个数组，数组[0]代表红外光，数组[1]代表红光
uint32_t win_data_bumber_now = 0;
int32_t win_flt_data_max_now[2] = {0,0};
int32_t win_flt_data_min_now[2] = {0,0};
int32_t win_dc_data_max_now[2] = {0,0};
int32_t win_dc_data_min_now[2] = {0,0};
uint32_t win_flt_data_ptp_value_last[2]= {0,0};
uint32_t win_dc_data_ptp_value_last[2]= {0,0};
int32_t red_dc_avg = 0;
int32_t ir_dc_avg = 0;
uint8_t dc_stable = 0;
int32_t sum_ir = 0;
int32_t sum_red = 0;


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

void SPO2_ResetWindow(void)
{
  win_data_bumber_now = 0;
  sum_ir = 0;
  sum_red = 0;
  for(int i = 0; i < 2; i++)
  {
    win_flt_data_ptp_value_last[i] = 0;
    win_dc_data_ptp_value_last[i] = 0;
  }
  ir_dc_avg = 0;
  red_dc_avg = 0;
  ef_R = 0;
  ef_num_ratio = 0;
  dc_stable = 0;
}



