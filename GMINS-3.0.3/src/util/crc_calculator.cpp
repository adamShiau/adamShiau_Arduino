#include "crc_calculator.h"

// 靜態成員變數定義
uint32_t CRCCalculator::crc32_table[256];
bool CRCCalculator::table_initialized = false;