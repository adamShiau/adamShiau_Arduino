#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "../domain/model/memory_manage.h"  // fog_parameter_t

/**
 * @brief Apply all configuration items from parameter container.
 *
 * Intended usage (boot restore):
 *   boot_capture_all(&g_fog_params);
 *   apply_configuration_from_container(&g_fog_params);
 */
void apply_configuration_from_container(const fog_parameter_t* params);

/**
 * @brief Apply datarate index (0..4) to system.
 *
 * - Sends CMD_SYNC_CNT to FPGA (ch=6)
 * - Updates MCU local sampling rate (set_data_rate + ahrs_attitude.init)
 */
bool apply_datarate_index(uint8_t dr_index);

/**
 * @brief Apply baudrate index (0..4) to MCU output port.
 *
 * - Only affects MCU UART (output port)
 */
bool apply_baudrate_index(uint8_t br_index);
