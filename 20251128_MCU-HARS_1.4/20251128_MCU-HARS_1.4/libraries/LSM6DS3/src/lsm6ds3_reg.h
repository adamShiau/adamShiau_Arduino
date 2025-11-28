/*
 ******************************************************************************
 * @file  lsm6d3s_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *        lsm6d3s_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LSM6DS3_REGS_H
#define LSM6DS3_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup LSM6DS3
  * @{
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

/**
  * @defgroup axisXbitXX_t
  * @brief    These unions are useful to represent different sensors data type.
  *           These unions are not need by the driver.
  *
  *           REMOVING the unions you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} axis3bit32_t;

typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;

/**
  * @}
  *
  */

typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
	*              You can create a sensor configuration by your own or using 
	*              Unico / Unicleo tools available on STMicroelectronics
	*              web site.
  *
  * @{
  *
  */

typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @addtogroup  LSM6DS3_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*lsm6d3s_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lsm6d3s_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
lsm6d3s_write_ptr  write_reg;
lsm6d3s_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} lsm6d3s_ctx_t;

/**
  * @}
  *
  */

/** @defgroup LSM6DS3 Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define LSM6DS3_I2C_ADD_L                    0xD5U
#define LSM6DS3_I2C_ADD_H                    0xD7U

/** Device Identification (Who am I) **/
#define LSM6DS3_ID                           0x6BU

/**
  * @}
  *
  */

#define LSM6DS3_PIN_CTRL                     0x02U
typedef struct {
  uint8_t not_used_01              : 6;
  uint8_t sdo_pu_en                : 1;
  uint8_t not_used_02              : 1;
} lsm6d3s_pin_ctrl_t;

#define LSM6DS3_FIFO_CTRL1                   0x07U
typedef struct {
  uint8_t wtm                      : 8;
} lsm6d3s_fifo_ctrl1_t;

#define LSM6DS3_FIFO_CTRL2                   0x08U
typedef struct {
  uint8_t wtm                      : 1;
  uint8_t not_used_01              : 3;
  uint8_t odrchg_en                : 1;
  uint8_t not_used_02              : 2;
  uint8_t stop_on_wtm              : 1;
} lsm6d3s_fifo_ctrl2_t;

#define LSM6DS3_FIFO_CTRL3                   0x09U
typedef struct {
  uint8_t bdr_xl                   : 4;
  uint8_t bdr_gy                   : 4;
} lsm6d3s_fifo_ctrl3_t;

#define LSM6DS3_FIFO_CTRL4                   0x0AU
typedef struct {
  uint8_t fifo_mode                : 3;
  uint8_t not_used_01              : 1;
  uint8_t odr_t_batch              : 2;
  uint8_t odr_ts_batch             : 2;
} lsm6d3s_fifo_ctrl4_t;

#define LSM6DS3_COUNTER_BDR_REG1             0x0BU
typedef struct {
  uint8_t cnt_bdr_th               : 3;
  uint8_t not_used_01              : 2;
  uint8_t trig_counter_bdr         : 1;
  uint8_t rst_counter_bdr          : 1;
  uint8_t dataready_pulsed         : 1;
} lsm6d3s_counter_bdr_reg1_t;

#define LSM6DS3_COUNTER_BDR_REG2             0x0CU
typedef struct {
  uint8_t cnt_bdr_th               : 8;
} lsm6d3s_counter_bdr_reg2_t;

#define LSM6DS3_INT1_CTRL                    0x0DU
typedef struct {
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t den_drdy_flag            : 1;
} lsm6d3s_int1_ctrl_t;

#define LSM6DS3_INT2_CTRL                    0x0EU
typedef struct {
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t not_used_01              : 1;
} lsm6d3s_int2_ctrl_t;

#define LSM6DS3_WHO_AM_I                     0x0FU
#define LSM6DS3_CTRL1_XL                     0x10U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
} lsm6d3s_ctrl1_xl_t;

#define LSM6DS3_CTRL2_G                      0x11U
typedef struct {
  uint8_t fs_g                     : 4; /* fs_4000 + fs_125 + fs_g */
  uint8_t odr_g                    : 4;
} lsm6d3s_ctrl2_g_t;

#define LSM6DS3_CTRL3_C                      0x12U
typedef struct {
  uint8_t sw_reset                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
} lsm6d3s_ctrl3_c_t;

#define LSM6DS3_CTRL4_C                      0x13U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_03              : 1;
} lsm6d3s_ctrl4_c_t;

#define LSM6DS3_CTRL5_C                      0x14U
typedef struct {
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t rounding                 : 2;
  uint8_t not_used_02              : 1;
} lsm6d3s_ctrl5_c_t;

#define LSM6DS3_CTRL6_G                      0x15U
typedef struct {
  uint8_t ftype                    : 3;
  uint8_t usr_off_w                : 1;
  uint8_t not_used_01              : 1;
  uint8_t den_mode                 : 3;   /* trig_en + lvl1_en + lvl2_en */
} lsm6d3s_ctrl6_g_t;

#define LSM6DS3_CTRL7_G                      0x16U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t usr_off_on_out           : 1;
  uint8_t not_used_02              : 2;
  uint8_t hpm_g                    : 2;
  uint8_t hp_en_g                  : 1;
  uint8_t not_used_03              : 1;
} lsm6d3s_ctrl7_g_t;

#define LSM6DS3_CTRL8_XL                     0x17U
typedef struct {
  uint8_t low_pass_on_6d           : 1;
  uint8_t not_used_01              : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t hpcf_xl                  : 3;
} lsm6d3s_ctrl8_xl_t;

#define LSM6DS3_CTRL9_XL                     0x18U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t device_conf              : 1;
  uint8_t den_lh                   : 1;
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
} lsm6d3s_ctrl9_xl_t;

#define LSM6DS3_CTRL10_C                     0x19U
typedef struct {
  uint8_t not_used_01              : 5;
  uint8_t timestamp_en             : 1;
  uint8_t not_used_02              : 2;
} lsm6d3s_ctrl10_c_t;

#define LSM6DS3_ALL_INT_SRC                  0x1AU
typedef struct {
  uint8_t ff_ia                    : 1;
  uint8_t wu_ia                    : 1;
  uint8_t not_used_01              : 2;
  uint8_t d6d_ia                   : 1;
  uint8_t sleep_change             : 1;
  uint8_t not_used_02              : 1;
  uint8_t timestamp_endcount       : 1;
} lsm6d3s_all_int_src_t;

#define LSM6DS3_WAKE_UP_SRC                  0x1BU
typedef struct {
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_change             : 1;
  uint8_t ff_ia                    : 1;
  uint8_t not_used_01              : 2;
} lsm6d3s_wake_up_src_t;

#define LSM6DS3_TAP_SRC                      0x1CU
typedef struct {
  uint8_t z_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_01              : 1;
} lsm6d3s_tap_src_t;

#define LSM6DS3_D6D_SRC                      0x1DU
typedef struct {
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
} lsm6d3s_d6d_src_t;

#define LSM6DS3_STATUS_REG                   0x1EU
typedef struct {
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
} lsm6d3s_status_reg_t;

#define LSM6DS3_OUT_TEMP_L                   0x20U
#define LSM6DS3_OUT_TEMP_H                   0x21U
#define LSM6DS3_OUTX_L_G                     0x22U
#define LSM6DS3_OUTX_H_G                     0x23U
#define LSM6DS3_OUTY_L_G                     0x24U
#define LSM6DS3_OUTY_H_G                     0x25U
#define LSM6DS3_OUTZ_L_G                     0x26U
#define LSM6DS3_OUTZ_H_G                     0x27U
#define LSM6DS3_OUTX_L_A                     0x28U
#define LSM6DS3_OUTX_H_A                     0x29U
#define LSM6DS3_OUTY_L_A                     0x2AU
#define LSM6DS3_OUTY_H_A                     0x2BU
#define LSM6DS3_OUTZ_L_A                     0x2CU
#define LSM6DS3_OUTZ_H_A                     0x2DU
#define LSM6DS3_FIFO_STATUS1                 0x3AU
typedef struct {
  uint8_t diff_fifo                : 8;
} lsm6d3s_fifo_status1_t;

#define LSM6DS3_FIFO_STATUS2                 0x3BU
typedef struct {
  uint8_t diff_fifo                : 2;
  uint8_t not_used_01              : 1;
  uint8_t over_run_latched         : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_wtm_ia              : 1;
} lsm6d3s_fifo_status2_t;

#define LSM6DS3_TIMESTAMP0                   0x40U
#define LSM6DS3_TIMESTAMP1                   0x41U
#define LSM6DS3_TIMESTAMP2                   0x42U
#define LSM6DS3_TIMESTAMP3                   0x43U
#define LSM6DS3_INT_CFG0                     0x56U
typedef struct {
  uint8_t lir                      : 1;
  uint8_t not_used_01              : 3;
  uint8_t slope_fds                : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t not_used_02              : 1;
} lsm6d3s_int_cfg0_t;

#define LSM6DS3_INT_CFG1                     0x58U
typedef struct {
  uint8_t not_used_01              : 5;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
} lsm6d3s_int_cfg1_t;

#define LSM6DS3_THS_6D                       0x59U
typedef struct {
  uint8_t not_used_01              : 5;
  uint8_t sixd_ths                 : 2;
  uint8_t d4d_en                   : 1;
} lsm6d3s_ths_6d_t;

#define LSM6DS3_INT_DUR2                     0x5AU
typedef struct {
  uint8_t shock                    : 2;
  uint8_t quiet                    : 2;
  uint8_t dur                      : 4;
} lsm6d3s_int_dur2_t;

#define LSM6DS3_WAKE_UP_THS                  0x5BU
typedef struct {
  uint8_t wk_ths                   : 6;
  uint8_t usr_off_on_wu            : 1;
  uint8_t not_used_01              : 1;
} lsm6d3s_wake_up_ths_t;

#define LSM6DS3_WAKE_UP_DUR                  0x5CU
typedef struct {
  uint8_t sleep_dur                : 4;
  uint8_t wake_ths_w               : 1;
  uint8_t wake_dur                 : 2;
  uint8_t ff_dur                   : 1;
} lsm6d3s_wake_up_dur_t;

#define LSM6DS3_FREE_FALL                    0x5DU
typedef struct {
  uint8_t ff_ths                   : 3;
  uint8_t ff_dur                   : 5;
} lsm6d3s_free_fall_t;

#define LSM6DS3_MD1_CFG                      0x5EU
typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t int1_6d                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t not_used_03              : 1;
  uint8_t int1_sleep_change        : 1;
} lsm6d3s_md1_cfg_t;

#define LSM6DS3_MD2_CFG                      0x5FU
typedef struct {
  uint8_t int2_timestamp           : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_6d                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t not_used_03              : 1;
  uint8_t int2_sleep_change        : 1;
} lsm6d3s_md2_cfg_t;

#define LSM6DS3_INTERNAL_FREQ_FINE           0x63U
typedef struct {
  uint8_t freq_fine                : 8;
} lsm6d3s_internal_freq_fine_t;

#define LSM6DS3_X_OFS_USR                    0x73U
#define LSM6DS3_Y_OFS_USR                    0x74U
#define LSM6DS3_Z_OFS_USR                    0x75U
#define LSM6DS3_FIFO_DATA_OUT_TAG            0x78U
typedef struct {
  uint8_t tag_parity               : 1;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_sensor               : 5;
} lsm6d3s_fifo_data_out_tag_t;

#define LSM6DS3_FIFO_DATA_OUT_X_L            0x79U
#define LSM6DS3_FIFO_DATA_OUT_X_H            0x7AU
#define LSM6DS3_FIFO_DATA_OUT_Y_L            0x7BU
#define LSM6DS3_FIFO_DATA_OUT_Y_H            0x7CU
#define LSM6DS3_FIFO_DATA_OUT_Z_L            0x7DU
#define LSM6DS3_FIFO_DATA_OUT_Z_H            0x7EU

/**
  * @defgroup LSM6DS3_Register_Union
  * @brief    This union group all the registers that has a bit-field
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union{
  lsm6d3s_pin_ctrl_t                      pin_ctrl;
  lsm6d3s_fifo_ctrl1_t                    fifo_ctrl1;
lsm6d3s_fifo_ctrl2_t                    fifo_ctrl2;
lsm6d3s_fifo_ctrl3_t                    fifo_ctrl3;
lsm6d3s_fifo_ctrl4_t                    fifo_ctrl4;
lsm6d3s_counter_bdr_reg1_t              counter_bdr_reg1;
lsm6d3s_counter_bdr_reg2_t              counter_bdr_reg2;
lsm6d3s_int1_ctrl_t                     int1_ctrl;
lsm6d3s_int2_ctrl_t                     int2_ctrl;
lsm6d3s_ctrl1_xl_t                      ctrl1_xl;
lsm6d3s_ctrl2_g_t                       ctrl2_g;
lsm6d3s_ctrl3_c_t                       ctrl3_c;
lsm6d3s_ctrl4_c_t                       ctrl4_c;
lsm6d3s_ctrl5_c_t                       ctrl5_c;
lsm6d3s_ctrl6_g_t                       ctrl6_g;
lsm6d3s_ctrl7_g_t                       ctrl7_g;
lsm6d3s_ctrl8_xl_t                      ctrl8_xl;
lsm6d3s_ctrl9_xl_t                      ctrl9_xl;
lsm6d3s_ctrl10_c_t                      ctrl10_c;
lsm6d3s_all_int_src_t                   all_int_src;
lsm6d3s_wake_up_src_t                   wake_up_src;
lsm6d3s_tap_src_t                       tap_src;
lsm6d3s_d6d_src_t                       d6d_src;
lsm6d3s_status_reg_t                    status_reg;
lsm6d3s_fifo_status1_t                  fifo_status1;
lsm6d3s_fifo_status2_t                  fifo_status2;
lsm6d3s_int_cfg0_t                      tap_cfg0;
lsm6d3s_int_cfg1_t                      int_cfg1;
lsm6d3s_ths_6d_t                        ths_6d;
lsm6d3s_int_dur2_t                      int_dur2;
lsm6d3s_wake_up_ths_t                   wake_up_ths;
lsm6d3s_wake_up_dur_t                   wake_up_dur;
lsm6d3s_free_fall_t                     free_fall;
lsm6d3s_md1_cfg_t                       md1_cfg;
lsm6d3s_md2_cfg_t                       md2_cfg;
lsm6d3s_internal_freq_fine_t            internal_freq_fine;
lsm6d3s_fifo_data_out_tag_t             fifo_data_out_tag;
  bitwise_t                                 bitwise;
  uint8_t                                   byte;
} lsm6d3s_reg_t;

/**
  * @}
  *
  */

int32_t lsm6d3s_read_reg(lsm6d3s_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lsm6d3s_write_reg(lsm6d3s_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float lsm6d3s_from_fs2g_to_mg(int16_t lsb);
extern float lsm6d3s_from_fs4g_to_mg(int16_t lsb);
extern float lsm6d3s_from_fs8g_to_mg(int16_t lsb);
extern float lsm6d3s_from_fs16g_to_mg(int16_t lsb);
extern float lsm6d3s_from_fs125dps_to_mdps(int16_t lsb);
extern float lsm6d3s_from_fs250dps_to_mdps(int16_t lsb);
extern float lsm6d3s_from_fs500dps_to_mdps(int16_t lsb);
extern float lsm6d3s_from_fs1000dps_to_mdps(int16_t lsb);
extern float lsm6d3s_from_fs2000dps_to_mdps(int16_t lsb);
extern float lsm6d3s_from_fs4000dps_to_mdps(int16_t lsb);
extern float lsm6d3s_from_lsb_to_celsius(int16_t lsb);
extern float lsm6d3s_from_lsb_to_nsec(int32_t lsb);

typedef enum {
  LSM6DS3_2g   = 0,
  LSM6DS3_16g  = 1, /* if XL_FS_MODE = ‘1’ -> LSM6DS3_2g */
  LSM6DS3_4g   = 2,
  LSM6DS3_8g   = 3,
} lsm6d3s_fs_xl_t;
int32_t lsm6d3s_xl_full_scale_set(lsm6d3s_ctx_t *ctx, lsm6d3s_fs_xl_t val);
int32_t lsm6d3s_xl_full_scale_get(lsm6d3s_ctx_t *ctx, lsm6d3s_fs_xl_t *val);

typedef enum {
  LSM6DS3_XL_ODR_OFF    = 0,
  LSM6DS3_XL_ODR_12Hz5  = 1,
  LSM6DS3_XL_ODR_26Hz   = 2,
  LSM6DS3_XL_ODR_52Hz   = 3,
  LSM6DS3_XL_ODR_104Hz  = 4,
  LSM6DS3_XL_ODR_208Hz  = 5,
  LSM6DS3_XL_ODR_417Hz  = 6,
  LSM6DS3_XL_ODR_833Hz  = 7,
  LSM6DS3_XL_ODR_1667Hz = 8,
  LSM6DS3_XL_ODR_3333Hz = 9,
  LSM6DS3_XL_ODR_6667Hz = 10,
  LSM6DS3_XL_ODR_6Hz5   = 11, /* (low power only) */
} lsm6d3s_odr_xl_t;
int32_t lsm6d3s_xl_data_rate_set(lsm6d3s_ctx_t *ctx, lsm6d3s_odr_xl_t val);
int32_t lsm6d3s_xl_data_rate_get(lsm6d3s_ctx_t *ctx, lsm6d3s_odr_xl_t *val);

typedef enum {
  LSM6DS3_125dps = 2,
  LSM6DS3_250dps = 0,
  LSM6DS3_500dps = 4,
  LSM6DS3_1000dps = 8,
  LSM6DS3_2000dps = 12,
  LSM6DS3_4000dps = 1,
} lsm6d3s_fs_g_t;
int32_t lsm6d3s_gy_full_scale_set(lsm6d3s_ctx_t *ctx, lsm6d3s_fs_g_t val);
int32_t lsm6d3s_gy_full_scale_get(lsm6d3s_ctx_t *ctx, lsm6d3s_fs_g_t *val);

typedef enum {
  LSM6DS3_GY_ODR_OFF    = 0,
  LSM6DS3_GY_ODR_12Hz5  = 1,
  LSM6DS3_GY_ODR_26Hz   = 2,
  LSM6DS3_GY_ODR_52Hz   = 3,
  LSM6DS3_GY_ODR_104Hz  = 4,
  LSM6DS3_GY_ODR_208Hz  = 5,
  LSM6DS3_GY_ODR_417Hz  = 6,
  LSM6DS3_GY_ODR_833Hz  = 7,
  LSM6DS3_GY_ODR_1667Hz = 8,
  LSM6DS3_GY_ODR_3333Hz = 9,
  LSM6DS3_GY_ODR_6667Hz = 10,
} lsm6d3s_odr_g_t;
int32_t lsm6d3s_gy_data_rate_set(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_odr_g_t val);
int32_t lsm6d3s_gy_data_rate_get(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_odr_g_t *val);

int32_t lsm6d3s_block_data_update_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_block_data_update_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_LSb_1mg  = 0,
  LSM6DS3_LSb_16mg = 1,
} lsm6d3s_usr_off_w_t;
int32_t lsm6d3s_xl_offset_weight_set(lsm6d3s_ctx_t *ctx,
                                   lsm6d3s_usr_off_w_t val);
int32_t lsm6d3s_xl_offset_weight_get(lsm6d3s_ctx_t *ctx,
                                   lsm6d3s_usr_off_w_t *val);

typedef struct {
lsm6d3s_all_int_src_t       all_int_src;
lsm6d3s_wake_up_src_t       wake_up_src;
lsm6d3s_tap_src_t           tap_src;
lsm6d3s_d6d_src_t           d6d_src;
lsm6d3s_status_reg_t        status_reg;
  } lsm6d3s_all_sources_t;
int32_t lsm6d3s_all_sources_get(lsm6d3s_ctx_t *ctx,
                              lsm6d3s_all_sources_t *val);

int32_t lsm6d3s_status_reg_get(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_status_reg_t *val);

int32_t lsm6d3s_xl_flag_data_ready_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_gy_flag_data_ready_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_temp_flag_data_ready_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_xl_usr_offset_x_set(lsm6d3s_ctx_t *ctx, uint8_t *buff);
int32_t lsm6d3s_xl_usr_offset_x_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_xl_usr_offset_y_set(lsm6d3s_ctx_t *ctx, uint8_t *buff);
int32_t lsm6d3s_xl_usr_offset_y_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_xl_usr_offset_z_set(lsm6d3s_ctx_t *ctx, uint8_t *buff);
int32_t lsm6d3s_xl_usr_offset_z_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_xl_usr_offset_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_xl_usr_offset_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_timestamp_rst(lsm6d3s_ctx_t *ctx);

int32_t lsm6d3s_timestamp_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_timestamp_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_timestamp_raw_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LSM6DS3_NO_ROUND      = 0,
  LSM6DS3_ROUND_XL      = 1,
  LSM6DS3_ROUND_GY      = 2,
  LSM6DS3_ROUND_GY_XL   = 3,
} lsm6d3s_rounding_t;
int32_t lsm6d3s_rounding_mode_set(lsm6d3s_ctx_t *ctx,
                                lsm6d3s_rounding_t val);
int32_t lsm6d3s_rounding_mode_get(lsm6d3s_ctx_t *ctx,
                                lsm6d3s_rounding_t *val);

int32_t lsm6d3s_temperature_raw_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_angular_rate_raw_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_acceleration_raw_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_fifo_out_raw_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_device_conf_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_device_conf_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_odr_cal_reg_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_odr_cal_reg_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_DRDY_LATCHED = 0,
  LSM6DS3_DRDY_PULSED  = 1,
} lsm6d3s_dataready_pulsed_t;
int32_t lsm6d3s_data_ready_mode_set(lsm6d3s_ctx_t *ctx,
                                  lsm6d3s_dataready_pulsed_t val);
int32_t lsm6d3s_data_ready_mode_get(lsm6d3s_ctx_t *ctx,
                                  lsm6d3s_dataready_pulsed_t *val);

int32_t lsm6d3s_device_id_get(lsm6d3s_ctx_t *ctx, uint8_t *buff);

int32_t lsm6d3s_reset_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_reset_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_auto_increment_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_auto_increment_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_boot_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_boot_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_XL_ST_DISABLE  = 0,
  LSM6DS3_XL_ST_POSITIVE = 1,
  LSM6DS3_XL_ST_NEGATIVE = 2,
} lsm6d3s_st_xl_t;
int32_t lsm6d3s_xl_self_test_set(lsm6d3s_ctx_t *ctx, lsm6d3s_st_xl_t val);
int32_t lsm6d3s_xl_self_test_get(lsm6d3s_ctx_t *ctx, lsm6d3s_st_xl_t *val);

typedef enum {
  LSM6DS3_GY_ST_DISABLE  = 0,
  LSM6DS3_GY_ST_POSITIVE = 1,
  LSM6DS3_GY_ST_NEGATIVE = 3,
} lsm6d3s_st_g_t;
int32_t lsm6d3s_gy_self_test_set(lsm6d3s_ctx_t *ctx, lsm6d3s_st_g_t val);
int32_t lsm6d3s_gy_self_test_get(lsm6d3s_ctx_t *ctx, lsm6d3s_st_g_t *val);

int32_t lsm6d3s_xl_filter_lp2_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_xl_filter_lp2_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_gy_filter_lp1_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_gy_filter_lp1_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_filter_settling_mask_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_filter_settling_mask_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_ULTRA_LIGHT  = 0,
  LSM6DS3_VERY_LIGHT   = 1,
  LSM6DS3_LIGHT        = 2,
  LSM6DS3_MEDIUM       = 3,
  LSM6DS3_STRONG       = 4,
  LSM6DS3_VERY_STRONG  = 5,
  LSM6DS3_AGGRESSIVE   = 6,
  LSM6DS3_XTREME       = 7,
} lsm6d3s_ftype_t;
int32_t lsm6d3s_gy_lp1_bandwidth_set(lsm6d3s_ctx_t *ctx, lsm6d3s_ftype_t val);
int32_t lsm6d3s_gy_lp1_bandwidth_get(lsm6d3s_ctx_t *ctx, lsm6d3s_ftype_t *val);

int32_t lsm6d3s_xl_lp2_on_6d_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_xl_lp2_on_6d_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_HP_PATH_DISABLE_ON_OUT    = 0x00,
  LSM6DS3_SLOPE_ODR_DIV_4           = 0x10,
  LSM6DS3_HP_ODR_DIV_10             = 0x11,
  LSM6DS3_HP_ODR_DIV_20             = 0x12,
  LSM6DS3_HP_ODR_DIV_45             = 0x13,
  LSM6DS3_HP_ODR_DIV_100            = 0x14,
  LSM6DS3_HP_ODR_DIV_200            = 0x15,
  LSM6DS3_HP_ODR_DIV_400            = 0x16,
  LSM6DS3_HP_ODR_DIV_800            = 0x17,
  LSM6DS3_HP_REF_MD_ODR_DIV_10      = 0x31,
  LSM6DS3_HP_REF_MD_ODR_DIV_20      = 0x32,
  LSM6DS3_HP_REF_MD_ODR_DIV_45      = 0x33,
  LSM6DS3_HP_REF_MD_ODR_DIV_100     = 0x34,
  LSM6DS3_HP_REF_MD_ODR_DIV_200     = 0x35,
  LSM6DS3_HP_REF_MD_ODR_DIV_400     = 0x36,
  LSM6DS3_HP_REF_MD_ODR_DIV_800     = 0x37,
  LSM6DS3_LP_ODR_DIV_10             = 0x01,
  LSM6DS3_LP_ODR_DIV_20             = 0x02,
  LSM6DS3_LP_ODR_DIV_45             = 0x03,
  LSM6DS3_LP_ODR_DIV_100            = 0x04,
  LSM6DS3_LP_ODR_DIV_200            = 0x05,
  LSM6DS3_LP_ODR_DIV_400            = 0x06,
  LSM6DS3_LP_ODR_DIV_800            = 0x07,
} lsm6d3s_hp_slope_xl_en_t;
int32_t lsm6d3s_xl_hp_path_on_out_set(lsm6d3s_ctx_t *ctx,
                                    lsm6d3s_hp_slope_xl_en_t val);
int32_t lsm6d3s_xl_hp_path_on_out_get(lsm6d3s_ctx_t *ctx,
                                    lsm6d3s_hp_slope_xl_en_t *val);

int32_t lsm6d3s_xl_fast_settling_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_xl_fast_settling_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_USE_SLOPE = 0,
  LSM6DS3_USE_HPF   = 1,
} lsm6d3s_slope_fds_t;
int32_t lsm6d3s_xl_hp_path_internal_set(lsm6d3s_ctx_t *ctx,
                                      lsm6d3s_slope_fds_t val);
int32_t lsm6d3s_xl_hp_path_internal_get(lsm6d3s_ctx_t *ctx,
                                      lsm6d3s_slope_fds_t *val);

typedef enum {
  LSM6DS3_HP_FILTER_NONE     = 0x00,
  LSM6DS3_HP_FILTER_16mHz    = 0x80,
  LSM6DS3_HP_FILTER_65mHz    = 0x81,
  LSM6DS3_HP_FILTER_260mHz   = 0x82,
  LSM6DS3_HP_FILTER_1Hz04    = 0x83,
} lsm6d3s_hpm_g_t;
int32_t lsm6d3s_gy_hp_path_internal_set(lsm6d3s_ctx_t *ctx,
                                      lsm6d3s_hpm_g_t val);
int32_t lsm6d3s_gy_hp_path_internal_get(lsm6d3s_ctx_t *ctx,
                                      lsm6d3s_hpm_g_t *val);

typedef enum {
  LSM6DS3_PULL_UP_DISC       = 0,
  LSM6DS3_PULL_UP_CONNECT    = 1,
} lsm6d3s_sdo_pu_en_t;
int32_t lsm6d3s_sdo_sa0_mode_set(lsm6d3s_ctx_t *ctx, lsm6d3s_sdo_pu_en_t val);
int32_t lsm6d3s_sdo_sa0_mode_get(lsm6d3s_ctx_t *ctx, lsm6d3s_sdo_pu_en_t *val);

typedef enum {
  LSM6DS3_SPI_4_WIRE = 0,
  LSM6DS3_SPI_3_WIRE = 1,
} lsm6d3s_sim_t;
int32_t lsm6d3s_spi_mode_set(lsm6d3s_ctx_t *ctx, lsm6d3s_sim_t val);
int32_t lsm6d3s_spi_mode_get(lsm6d3s_ctx_t *ctx, lsm6d3s_sim_t *val);

typedef enum {
  LSM6DS3_I2C_ENABLE  = 0,
  LSM6DS3_I2C_DISABLE = 1,
} lsm6d3s_i2c_disable_t;
int32_t lsm6d3s_i2c_interface_set(lsm6d3s_ctx_t *ctx,
                                lsm6d3s_i2c_disable_t val);
int32_t lsm6d3s_i2c_interface_get(lsm6d3s_ctx_t *ctx,
                                lsm6d3s_i2c_disable_t *val);

typedef struct {
  lsm6d3s_int1_ctrl_t          int1_ctrl;
  lsm6d3s_md1_cfg_t            md1_cfg;
} lsm6d3s_pin_int1_route_t;
int32_t lsm6d3s_pin_int1_route_set(lsm6d3s_ctx_t *ctx,
                                 lsm6d3s_pin_int1_route_t *val);
int32_t lsm6d3s_pin_int1_route_get(lsm6d3s_ctx_t *ctx,
                                 lsm6d3s_pin_int1_route_t *val);

typedef struct {
lsm6d3s_int2_ctrl_t          int2_ctrl;
lsm6d3s_md2_cfg_t            md2_cfg;
} lsm6d3s_pin_int2_route_t;
int32_t lsm6d3s_pin_int2_route_set(lsm6d3s_ctx_t *ctx,
                                 lsm6d3s_pin_int2_route_t *val);
int32_t lsm6d3s_pin_int2_route_get(lsm6d3s_ctx_t *ctx,
                                 lsm6d3s_pin_int2_route_t *val);

typedef enum {
  LSM6DS3_PUSH_PULL   = 0,
  LSM6DS3_OPEN_DRAIN  = 1,
} lsm6d3s_pp_od_t;
int32_t lsm6d3s_pin_mode_set(lsm6d3s_ctx_t *ctx, lsm6d3s_pp_od_t val);
int32_t lsm6d3s_pin_mode_get(lsm6d3s_ctx_t *ctx, lsm6d3s_pp_od_t *val);

typedef enum {
  LSM6DS3_ACTIVE_HIGH = 0,
  LSM6DS3_ACTIVE_LOW  = 1,
} lsm6d3s_h_lactive_t;
int32_t lsm6d3s_pin_polarity_set(lsm6d3s_ctx_t *ctx, lsm6d3s_h_lactive_t val);
int32_t lsm6d3s_pin_polarity_get(lsm6d3s_ctx_t *ctx, lsm6d3s_h_lactive_t *val);

int32_t lsm6d3s_all_on_int1_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_all_on_int1_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_ALL_INT_PULSED            = 0,
  LSM6DS3_ALL_INT_LATCHED           = 3,
} lsm6d3s_lir_t;
int32_t lsm6d3s_int_notification_set(lsm6d3s_ctx_t *ctx, lsm6d3s_lir_t val);
int32_t lsm6d3s_int_notification_get(lsm6d3s_ctx_t *ctx, lsm6d3s_lir_t *val);

typedef enum {
  LSM6DS3_LSb_FS_DIV_64       = 0,
  LSM6DS3_LSb_FS_DIV_256      = 1,
} lsm6d3s_wake_ths_w_t;
int32_t lsm6d3s_wkup_ths_weight_set(lsm6d3s_ctx_t *ctx,
                                  lsm6d3s_wake_ths_w_t val);
int32_t lsm6d3s_wkup_ths_weight_get(lsm6d3s_ctx_t *ctx,
                                  lsm6d3s_wake_ths_w_t *val);

int32_t lsm6d3s_wkup_threshold_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_wkup_threshold_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_xl_usr_offset_on_wkup_set(lsm6d3s_ctx_t *ctx,
                                          uint8_t val);
int32_t lsm6d3s_xl_usr_offset_on_wkup_get(lsm6d3s_ctx_t *ctx,
                                          uint8_t *val);

int32_t lsm6d3s_wkup_dur_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_wkup_dur_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_gy_sleep_mode_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_gy_sleep_mode_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_DRIVE_SLEEP_CHG_EVENT = 0,
  LSM6DS3_DRIVE_SLEEP_STATUS    = 1,
} lsm6d3s_sleep_status_on_int_t;
int32_t lsm6d3s_act_pin_notification_set(lsm6d3s_ctx_t *ctx,
                                       lsm6d3s_sleep_status_on_int_t val);
int32_t lsm6d3s_act_pin_notification_get(lsm6d3s_ctx_t *ctx,
                                       lsm6d3s_sleep_status_on_int_t *val);

typedef enum {
  LSM6DS3_XL_AND_GY_NOT_AFFECTED      = 0,
  LSM6DS3_XL_12Hz5_GY_NOT_AFFECTED    = 1,
  LSM6DS3_XL_12Hz5_GY_SLEEP           = 2,
  LSM6DS3_XL_12Hz5_GY_PD              = 3,
} lsm6d3s_inact_en_t;
int32_t lsm6d3s_act_mode_set(lsm6d3s_ctx_t *ctx,
                           lsm6d3s_inact_en_t val);
int32_t lsm6d3s_act_mode_get(lsm6d3s_ctx_t *ctx,
                           lsm6d3s_inact_en_t *val);

int32_t lsm6d3s_act_sleep_dur_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_act_sleep_dur_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_DEG_80  = 0,
  LSM6DS3_DEG_70  = 1,
  LSM6DS3_DEG_60  = 2,
  LSM6DS3_DEG_50  = 3,
} lsm6d3s_sixd_ths_t;
int32_t lsm6d3s_6d_threshold_set(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_sixd_ths_t val);
int32_t lsm6d3s_6d_threshold_get(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_sixd_ths_t *val);

int32_t lsm6d3s_4d_mode_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_4d_mode_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_FF_TSH_156mg = 0,
  LSM6DS3_FF_TSH_219mg = 1,
  LSM6DS3_FF_TSH_250mg = 2,
  LSM6DS3_FF_TSH_312mg = 3,
  LSM6DS3_FF_TSH_344mg = 4,
  LSM6DS3_FF_TSH_406mg = 5,
  LSM6DS3_FF_TSH_469mg = 6,
  LSM6DS3_FF_TSH_500mg = 7,
} lsm6d3s_ff_ths_t;
int32_t lsm6d3s_ff_threshold_set(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_ff_ths_t val);
int32_t lsm6d3s_ff_threshold_get(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_ff_ths_t *val);

int32_t lsm6d3s_ff_dur_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_ff_dur_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_fifo_watermark_set(lsm6d3s_ctx_t *ctx, uint16_t val);
int32_t lsm6d3s_fifo_watermark_get(lsm6d3s_ctx_t *ctx, uint16_t *val);

int32_t lsm6d3s_fifo_virtual_sens_odr_chg_set(lsm6d3s_ctx_t *ctx,
                                              uint8_t val);
int32_t lsm6d3s_fifo_virtual_sens_odr_chg_get(lsm6d3s_ctx_t *ctx,
                                              uint8_t *val);

int32_t lsm6d3s_fifo_stop_on_wtm_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_fifo_stop_on_wtm_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_XL_NOT_BATCHED        =  0,
  LSM6DS3_XL_BATCHED_AT_12Hz5   =  1,
  LSM6DS3_XL_BATCHED_AT_26Hz    =  2,
  LSM6DS3_XL_BATCHED_AT_52Hz    =  3,
  LSM6DS3_XL_BATCHED_AT_104Hz   =  4,
  LSM6DS3_XL_BATCHED_AT_208Hz   =  5,
  LSM6DS3_XL_BATCHED_AT_417Hz   =  6,
  LSM6DS3_XL_BATCHED_AT_833Hz   =  7,
  LSM6DS3_XL_BATCHED_AT_1667Hz  =  8,
  LSM6DS3_XL_BATCHED_AT_3333Hz  =  9,
  LSM6DS3_XL_BATCHED_AT_6667Hz  = 10,
  LSM6DS3_XL_BATCHED_AT_6Hz5    = 11,
} lsm6d3s_bdr_xl_t;
int32_t lsm6d3s_fifo_xl_batch_set(lsm6d3s_ctx_t *ctx, lsm6d3s_bdr_xl_t val);
int32_t lsm6d3s_fifo_xl_batch_get(lsm6d3s_ctx_t *ctx, lsm6d3s_bdr_xl_t *val);

typedef enum {
  LSM6DS3_GY_NOT_BATCHED         = 0,
  LSM6DS3_GY_BATCHED_AT_12Hz5    = 1,
  LSM6DS3_GY_BATCHED_AT_26Hz     = 2,
  LSM6DS3_GY_BATCHED_AT_52Hz     = 3,
  LSM6DS3_GY_BATCHED_AT_104Hz    = 4,
  LSM6DS3_GY_BATCHED_AT_208Hz    = 5,
  LSM6DS3_GY_BATCHED_AT_417Hz    = 6,
  LSM6DS3_GY_BATCHED_AT_833Hz    = 7,
  LSM6DS3_GY_BATCHED_AT_1667Hz   = 8,
  LSM6DS3_GY_BATCHED_AT_3333Hz   = 9,
  LSM6DS3_GY_BATCHED_AT_6667Hz   = 10,
  LSM6DS3_GY_BATCHED_6Hz5        = 11,
} lsm6d3s_bdr_gy_t;
int32_t lsm6d3s_fifo_gy_batch_set(lsm6d3s_ctx_t *ctx, lsm6d3s_bdr_gy_t val);
int32_t lsm6d3s_fifo_gy_batch_get(lsm6d3s_ctx_t *ctx, lsm6d3s_bdr_gy_t *val);

typedef enum {
  LSM6DS3_BYPASS_MODE             = 0,
  LSM6DS3_FIFO_MODE               = 1,
  LSM6DS3_STREAM_TO_FIFO_MODE     = 3,
  LSM6DS3_BYPASS_TO_STREAM_MODE   = 4,
  LSM6DS3_STREAM_MODE             = 6,
  LSM6DS3_BYPASS_TO_FIFO_MODE     = 7,
} lsm6d3s_fifo_mode_t;
int32_t lsm6d3s_fifo_mode_set(lsm6d3s_ctx_t *ctx, lsm6d3s_fifo_mode_t val);
int32_t lsm6d3s_fifo_mode_get(lsm6d3s_ctx_t *ctx, lsm6d3s_fifo_mode_t *val);

typedef enum {
  LSM6DS3_TEMP_NOT_BATCHED        = 0,
  LSM6DS3_TEMP_BATCHED_AT_52Hz    = 1,
  LSM6DS3_TEMP_BATCHED_AT_12Hz5   = 2,
  LSM6DS3_TEMP_BATCHED_AT_1Hz6    = 3,
} lsm6d3s_odr_t_batch_t;
int32_t lsm6d3s_fifo_temp_batch_set(lsm6d3s_ctx_t *ctx,
                                  lsm6d3s_odr_t_batch_t val);
int32_t lsm6d3s_fifo_temp_batch_get(lsm6d3s_ctx_t *ctx,
                                  lsm6d3s_odr_t_batch_t *val);

typedef enum {
  LSM6DS3_NO_DECIMATION = 0,
  LSM6DS3_DEC_1         = 1,
  LSM6DS3_DEC_8         = 2,
  LSM6DS3_DEC_32        = 3,
} lsm6d3s_odr_ts_batch_t;
int32_t lsm6d3s_fifo_timestamp_decimation_set(lsm6d3s_ctx_t *ctx,
                                            lsm6d3s_odr_ts_batch_t val);
int32_t lsm6d3s_fifo_timestamp_decimation_get(lsm6d3s_ctx_t *ctx,
                                            lsm6d3s_odr_ts_batch_t *val);

typedef enum {
  LSM6DS3_XL_BATCH_EVENT   = 0,
  LSM6DS3_GYRO_BATCH_EVENT = 1,
} lsm6d3s_trig_counter_bdr_t;
int32_t lsm6d3s_fifo_cnt_event_batch_set(lsm6d3s_ctx_t *ctx,
                                       lsm6d3s_trig_counter_bdr_t val);
int32_t lsm6d3s_fifo_cnt_event_batch_get(lsm6d3s_ctx_t *ctx,
                                       lsm6d3s_trig_counter_bdr_t *val);

int32_t lsm6d3s_rst_batch_counter_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_rst_batch_counter_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_batch_counter_threshold_set(lsm6d3s_ctx_t *ctx,
                                            uint16_t val);
int32_t lsm6d3s_batch_counter_threshold_get(lsm6d3s_ctx_t *ctx,
                                            uint16_t *val);

int32_t lsm6d3s_fifo_data_level_get(lsm6d3s_ctx_t *ctx, uint16_t *val);

int32_t lsm6d3s_fifo_status_get(lsm6d3s_ctx_t *ctx,
                              lsm6d3s_fifo_status2_t *val);

int32_t lsm6d3s_fifo_full_flag_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_fifo_ovr_flag_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_fifo_wtm_flag_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DS3_GYRO_NC_TAG    = 1,
  LSM6DS3_XL_NC_TAG,
  LSM6DS3_TEMPERATURE_TAG,
  LSM6DS3_TIMESTAMP_TAG,
  LSM6DS3_CFG_CHANGE_TAG,
} lsm6d3s_fifo_tag_t;
int32_t lsm6d3s_fifo_sensor_tag_get(lsm6d3s_ctx_t *ctx,
                                  lsm6d3s_fifo_tag_t *val);

typedef enum {
  LSM6DS3_DEN_DISABLE    = 0,
  LSM6DS3_LEVEL_FIFO     = 6,
  LSM6DS3_LEVEL_LETCHED  = 3,
  LSM6DS3_LEVEL_TRIGGER  = 2,
  LSM6DS3_EDGE_TRIGGER   = 4,
} lsm6d3s_den_mode_t;
int32_t lsm6d3s_den_mode_set(lsm6d3s_ctx_t *ctx,
                           lsm6d3s_den_mode_t val);
int32_t lsm6d3s_den_mode_get(lsm6d3s_ctx_t *ctx,
                           lsm6d3s_den_mode_t *val);

typedef enum {
  LSM6DS3_DEN_ACT_LOW  = 0,
  LSM6DS3_DEN_ACT_HIGH = 1,
} lsm6d3s_den_lh_t;
int32_t lsm6d3s_den_polarity_set(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_den_lh_t val);
int32_t lsm6d3s_den_polarity_get(lsm6d3s_ctx_t *ctx,
                               lsm6d3s_den_lh_t *val);

typedef enum {
  LSM6DS3_STAMP_IN_GY_DATA     = 0,
  LSM6DS3_STAMP_IN_XL_DATA     = 1,
  LSM6DS3_STAMP_IN_GY_XL_DATA  = 2,
} lsm6d3s_den_xl_g_t;
int32_t lsm6d3s_den_enable_set(lsm6d3s_ctx_t *ctx,
                             lsm6d3s_den_xl_g_t val);
int32_t lsm6d3s_den_enable_get(lsm6d3s_ctx_t *ctx,
                             lsm6d3s_den_xl_g_t *val);

int32_t lsm6d3s_den_mark_axis_x_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_den_mark_axis_x_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_den_mark_axis_y_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_den_mark_axis_y_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

int32_t lsm6d3s_den_mark_axis_z_set(lsm6d3s_ctx_t *ctx, uint8_t val);
int32_t lsm6d3s_den_mark_axis_z_get(lsm6d3s_ctx_t *ctx, uint8_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* LSM6DS3_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
