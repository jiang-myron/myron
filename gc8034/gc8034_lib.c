/* gc8034_lib.c
 *
 *Copyright (c) 2017 Qualcomm Technologies, Inc.
 *All Rights Reserved.
 *Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <stdio.h>
#include "gc8034_lib.h"
#include "debug_lib.h"
#include <string.h>
#include "sensor_lib.h"
#include <utils/Log.h>

/**
 * FUNCTION: sensor_real_to_register_gain
 *
 * DESCRIPTION: Calcuate the sensor exposure
 **/
static unsigned int sensor_real_to_register_gain(float gain)
{
  uint16_t reg_gain;

  if (gain < GC8034MinRealGain)
    gain = GC8034MinRealGain;
  if (gain > GC8034MaxRealGain)
    gain = GC8034MaxRealGain;
  ALOGE("gc8034,real_gain=%f", gain);
  reg_gain = (uint16_t)(gain * 64.0f);

  return reg_gain;
}

/**
 * FUNCTION: sensor_register_to_real_gain
 *
 * DESCRIPTION: Calcuate the sensor exposure
 **/
static float sensor_register_to_real_gain(unsigned int reg_gain)
{
  float gain;

  if (reg_gain > GC8034MaxRegisterGain)
    reg_gain = GC8034MaxRegisterGain;
  else if (reg_gain <= GC8034MinRegisterGain)
    reg_gain = GC8034MinRegisterGain;
  ALOGE("gc8034 register_gain=%u", reg_gain);

  gain = (float)(reg_gain / 64.0f);
  return gain;
}

/**
 * FUNCTION: sensor_calculate_exposure
 *
 * DESCRIPTION: Calcuate the sensor exposure
 **/
static int sensor_calculate_exposure(float real_gain,
  unsigned int line_count, sensor_exposure_info_t *exp_info,
  __attribute__((unused)) float s_real_gain)
{
  if (!exp_info) {
    return -1;
  }

  ALOGE("GC8034 reg_gain: %d, real_gain: %f, line_cnt: %d\n",
    sensor_real_to_register_gain(real_gain),
    sensor_register_to_real_gain(exp_info->reg_gain),
    line_count);
  exp_info->reg_gain = sensor_real_to_register_gain(real_gain);
  exp_info->sensor_real_gain =
    sensor_register_to_real_gain(exp_info->reg_gain);
  exp_info->digital_gain = real_gain / exp_info->sensor_real_gain;
  exp_info->line_count = line_count;
  return 0;

}

/**
 * FUNCTION: sensor_fill_exposure_array
 *
 * DESCRIPTION: Fill the sensor exposure array
 **/
static int sensor_fill_exposure_array(unsigned int gain,
  __attribute__((unused))unsigned int digital_gain, unsigned int line,
  __attribute__((unused)) unsigned int fl_lines, __attribute__((unused)) int luma_avg,
  __attribute__((unused)) unsigned int fgain,
  struct camera_i2c_reg_setting* reg_setting,
  __attribute__((unused)) unsigned int s_gain,
  __attribute__((unused)) int s_linecount,
  __attribute__((unused)) int is_hdr_enabled)
{
  int32_t i, rc = 0;
  int16_t gain_index = 0;
  uint16_t reg_count = 0;
  uint16_t gain_b6 = 0, gain_b1 = 1, gain_b2 = 0;
  uint32_t temp_gain = 0, Dgain_ratio = 256;
  uint8_t cur_res = 0, BorF = 0;
  int16_t temp_flines = 0, cal_line = 0;

  ALOGE("GC8034,fl_lines=%d,gain=%d, line=%d\n", fl_lines, gain, line);

  if (1 == line && gain <= 64)
    return rc;

  if (!reg_setting) {
    return -1;
  }

  cur_res = fl_lines >> 16;
  ALOGE("GC8034,cur_res=%d\n", cur_res);
  switch (cur_res) {
  case 1:BorF = 0;
    break;
  default:BorF = 1;
    break;
  }

  /***************************Shutter Start*********************************/
  line = line < 4 ? 4 : line; /*anti color deviation on shot expoure*/

  cal_line = line >> 1;
  cal_line = cal_line << 1;
  Dgain_ratio = 256 * line / cal_line;

  temp_flines = fl_lines & 0xffff;
  ALOGE("GC8034,temp_flines1=%d\n", temp_flines);
  temp_flines = temp_flines >> 1;
  temp_flines = temp_flines << 1;
  temp_flines = temp_flines < Min_FL[cur_res] ? Min_FL[cur_res] : temp_flines;
  temp_flines = temp_flines - Basic_Line[cur_res];
  ALOGE("GC8034,outinf_fl_lines=%d,Basic_Line=%d\n", sensor_lib_ptr.out_info_array.out_info[cur_res].frame_length_lines, Basic_Line[cur_res]);
  ALOGE("GC8034,temp_flines2=%d, cal_line=%d\n", temp_flines, cal_line);

  reg_setting->reg_setting[reg_count].reg_addr =
    sensor_lib_ptr.exp_gain_info.coarse_int_time_addr;
  reg_setting->reg_setting[reg_count].reg_data = (cal_line & 0x7F00) >> 8;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr =
    sensor_lib_ptr.exp_gain_info.coarse_int_time_addr + 1;
  reg_setting->reg_setting[reg_count].reg_data = cal_line & 0xFF;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0x07;
  reg_setting->reg_setting[reg_count].reg_data = (temp_flines & 0x1F00) >> 8;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0x08;
  reg_setting->reg_setting[reg_count].reg_data = temp_flines & 0xFF;
  reg_count++;

  /*****************************Shutter End********************************/

  /***************************GIAN START*********************************/
  for (gain_index = MEAG_INDEX - 1; gain_index >= 0; gain_index--)
    if (gain >= gain_level[gain_index]) {
      gain_b6 = gain_index;
      temp_gain = 256 * gain / gain_level[gain_index];
      temp_gain = temp_gain * Dgain_ratio / 256;
      gain_b1 = temp_gain >> 8;
      gain_b2 = temp_gain & 0xff;
      reg_setting->reg_setting[reg_count].reg_addr = 0xfe;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][0];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0x20;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][1];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0x33;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][2];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xfe;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][3];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xdf;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][4];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xe7;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][5];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xe8;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][6];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xe9;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][7];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xea;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][8];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xeb;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][9];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xec;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][10];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xed;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][11];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xee;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][12];
      reg_count++;
      reg_setting->reg_setting[reg_count].reg_addr = 0xfe;
      reg_setting->reg_setting[reg_count].reg_data = agc_register[BorF * MAX_AG_INDEX + gain_index][13];
      reg_count++;
      break;
    }

  ALOGE("GC8034,gain=%d,gain_b6=%d,gain_level=%d\n", gain, gain_b6, gain_level[gain_index]);
  ALOGE("GC8034,gain_b1=%d,gain_b2=%d\n", gain_b1, gain_b2);

  reg_setting->reg_setting[reg_count].reg_addr =
    sensor_lib_ptr.exp_gain_info.global_gain_addr + 4;
  reg_setting->reg_setting[reg_count].reg_data = gain_b6;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr =
    sensor_lib_ptr.exp_gain_info.global_gain_addr - 1;
  reg_setting->reg_setting[reg_count].reg_data = gain_b1;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr =
    sensor_lib_ptr.exp_gain_info.global_gain_addr;
  reg_setting->reg_setting[reg_count].reg_data = gain_b2;
  reg_count++;
  /***************************GIAN END*********************************/

  reg_setting->size = reg_count;
  reg_setting->addr_type = CAMERA_I2C_BYTE_ADDR;
  reg_setting->data_type = CAMERA_I2C_BYTE_DATA;
  reg_setting->delay = 0;

  return rc;
}

/**
 * FUNCTION: sensor_open_lib
 *
 * DESCRIPTION: Open sensor library and returns data pointer
 **/
void *sensor_open_lib(void)
{
  return &sensor_lib_ptr;
}
