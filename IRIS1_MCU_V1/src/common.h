#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
/*** for serial_printf() impl */
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "memory_manage.h"



#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_PRINT

#ifdef DEBUG_PRINT
    #define DEBUG_PRINT(...) serial_printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)
#endif

extern const uint8_t HDR_ABBA[2];
extern const uint8_t TRL_5556[2];
extern const uint8_t HDR_OUT[4];

/* ---------- Protocol condition values ----------
 * Keep aligned with readDataDynamic():
 *   1 -> AB BA ... 55 56  (DATA1_SIZE = 6)
 *   2 -> CD DC ... 57 58  (DATA2_SIZE = 13)
 *   3 -> EF FE ... 53 54  (DATA3_SIZE = 6)
 */
typedef enum {
  RX_CONDITION_INIT         = 0,
  RX_CONDITION_ABBA_5556    = 1,
  RX_CONDITION_CDDC_5758    = 2,
  RX_CONDITION_EFFE_5354    = 3
} rx_condition_t;

/* ---------- MUX / State selections (adjust if needed) ---------- */
typedef enum {
  MUX_ESCAPE     = 0,
  MUX_PARAMETER  = 1,
  MUX_OUTPUT     = 2,
  MUX_DEFAULT    = 3
} mux_t;

typedef enum {
    SEL_IDLE = 0,
    SEL_RST = 1,
    SEL_FOG = 2,
    SEL_FOG_2 = 3,
    SEL_FOG_3 = 4,
    SEL_IMU = 5,
    SEL_NMEA = 6,
    SEL_FOG_PARA = 7,
    SEL_HP_TEST = 8,
    SEL_ATT_NMEA = 9
} select_fn_t;

typedef enum {
    MODE_RST = 0,
    MODE_FOG = 1,
    MODE_IMU = 2
} output_mode_t;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  int32_t int_val;
}
my_float_t;

/*** sensor data structure delaration */

typedef struct {
  my_float_t err; 
  my_float_t step;     
} fog_component_t;

typedef struct {
  fog_component_t fogx;
  fog_component_t fogy;
  fog_component_t fogz;
} fog_t;

typedef struct {
  my_float_t tempx;  
  my_float_t tempy; 
  my_float_t tempz; 
} temp_t;

typedef struct {
  my_float_t time;  
} my_time_t;

typedef struct {
  my_float_t ax;  
  my_float_t ay; 
  my_float_t az; 
  my_float_t temp;
} accl_t;

typedef struct 
{
  my_time_t time;
  fog_t fog;
  temp_t temp;
  accl_t adxl357;
}my_sensor_t;

typedef struct {
  my_float_t x;
  my_float_t y;
  my_float_t z;
} calibrated_data_t;

/* ---------- Command control structure ---------- */
typedef struct
{
  rx_condition_t condition;  /* 0/1/2/3 per above */
  uint8_t  SN[13];           /* Up to 12 chars + NUL */
  uint8_t  complete;         /* 1 when a command parsed successfully */
  uint8_t  mux;              /* MUX_ESCAPE / MUX_OUTPUT / MUX_PARAMETER */
  uint8_t  select_fn;        /* SEL_IDLE or others you add later */
  uint8_t  ch;               /* channel */
  uint8_t  cmd;              /* command code */
  uint8_t  run;              /* optional run flag */
  int32_t  value;            /* 32-bit value, big-endian assembled */
} cmd_ctrl_t;

/* ---------- API ---------- */
void get_uart_cmd(uint8_t* data, cmd_ctrl_t* rx);
void cmd_mux(cmd_ctrl_t* rx);
void fog_parameter(cmd_ctrl_t*, fog_parameter_t*);
size_t sendCmd(Print& port, const uint8_t header[2], const uint8_t trailer[2], uint8_t cmd, 
    int32_t value, uint8_t ch);
void update_parameter_container(const cmd_ctrl_t* rx, fog_parameter_t* fog_inst, uint8_t idx);

/*** Key-value callback interface 型別宣告 */
typedef void (*kv_cb_t)(int key, int32_t val, void* ctx);

void dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch);
void dump_misalignment_param(fog_parameter_t* fog_inst);
size_t read_json_object(Stream& s, char* out, size_t out_cap, uint32_t timeout_ms);
void parse_simple_json_ints(const char* js, kv_cb_t cb, void* ctx);
void dumpPkt(uint8_t *pkt, int len);
int update_raw_data(const uint8_t* pkt, my_sensor_t* out);

/* ===================== API: printf-like serial output ===================== */
/* C-linkage printf wrappers (you can call these from C or C++). */
int  serial_printf(const char* fmt, ...);
int  serial_vprintf(const char* fmt, va_list ap);

#ifdef __cplusplus
} /* extern "C" */
#endif

/* This requires C++ (uses Arduino Print). Provide only in C++ compilation. */
#ifdef __cplusplus
#include <Print.h>

/**
 * @brief Set output stream for serial_printf() (default: &Serial).
 *        You can pass &Serial1 / &Serial4 / any Print*.
 */
void serial_set_stream(Print* s);
#endif


#endif /* COMMON_H */
