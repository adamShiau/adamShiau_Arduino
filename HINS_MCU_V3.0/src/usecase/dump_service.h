#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "../domain/model/memory_manage.h"   // for fog_parameter_t

// 對外 API（你現有的 dump_* 最終都會落在這裡）
bool dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch);   // ch=1/2/3
bool dump_misalignment_param(fog_parameter_t* fog_inst);      // ch=4
bool dump_SN(fog_parameter_t* fog_inst);                      // ch=5
bool dump_version(fog_parameter_t* fog_inst);                 // ch=7（不存，forward payload）
bool boot_capture_all(fog_parameter_t* fog_inst);
