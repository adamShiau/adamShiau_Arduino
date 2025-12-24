#pragma once
#include <Arduino.h>
#include "../common.h"   // temporary: provides cmd_ctrl_t, fog_parameter_t, constants, sendCmd, etc.

/**
 * @brief Handle parameter-related commands (usecase layer).
 *
 * Caller provides the Print stream to use for sending commands.
 * This function does NOT perform UART framing/reading; it only interprets an already-decoded cmd_ctrl_t.
 */
void parameter_service_handle(Print& port, cmd_ctrl_t* rx, fog_parameter_t* fog_inst);
