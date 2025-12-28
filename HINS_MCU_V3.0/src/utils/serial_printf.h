#pragma once
#include <Arduino.h>
#include <stdarg.h>

// printf-like output to a selectable Print stream (default: &Serial)
int  serial_printf(const char* fmt, ...);
int  serial_vprintf(const char* fmt, va_list ap);
void serial_set_stream(Print* s);

