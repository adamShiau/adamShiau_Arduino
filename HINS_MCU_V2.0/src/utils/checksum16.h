#pragma once
#include <stdint.h>
#include <stddef.h>

// Fletcher-16 over data[0..len-1]
uint16_t fletcher16(const uint8_t* data, size_t len);
