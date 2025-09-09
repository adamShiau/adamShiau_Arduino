#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

struct TxStats {
  uint32_t try_bytes = 0;
  uint32_t sent_bytes = 0;
  uint32_t drop_packets = 0;
  uint32_t last_report_ms = 0;
};

inline bool uart_try_write(const uint8_t* buf, uint16_t len, TxStats& s) {
  s.try_bytes += len;
  if (Serial.availableForWrite() < len) { s.drop_packets++; return false; }
  size_t n = Serial.write(buf, len);
  s.sent_bytes += n;
  return n == len;
}

inline void uart_report_every_5s(TxStats& s) {
  uint32_t now = millis();
  if (now - s.last_report_ms >= 5000u) {
    s.last_report_ms = now;
    // Arduino Serial沒有printf，使用sprintf + print
    char buffer[128];
    sprintf(buffer, "TX 5s: try=%luB sent=%luB drop=%lu",
            (unsigned long)s.try_bytes, (unsigned long)s.sent_bytes,
            (unsigned long)s.drop_packets);
    LOGI("%s", buffer);
    s.try_bytes = s.sent_bytes = s.drop_packets = 0;
  }
}