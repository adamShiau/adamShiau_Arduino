#include "serial_printf.h"

#ifndef SERIAL_PRINTF_STACKBUF
#define SERIAL_PRINTF_STACKBUF 128   // Small messages use stack buffer
#endif

#ifndef SERIAL_PRINTF_ALLOW_HEAP
#define SERIAL_PRINTF_ALLOW_HEAP 1   // Large messages use heap if needed
#endif

// Default output: USB Serial. You can change with serial_set_stream().
static Print* g_serial_out = &Serial;

void serial_set_stream(Print* s)
{
  if (s) g_serial_out = s;
}

int serial_vprintf(const char* fmt, va_list ap)
{
  if (!g_serial_out) return 0;

  char  stackbuf[SERIAL_PRINTF_STACKBUF];
  char* buf = stackbuf;
  int   n;

  // First try: format into stack buffer
  va_list ap2;
  va_copy(ap2, ap);
  n = vsnprintf(stackbuf, sizeof(stackbuf), fmt, ap2);
  va_end(ap2);

  if (n < 0) return n;

  // If it fits, write and return
  if (n < (int)sizeof(stackbuf))
  {
    g_serial_out->write((const uint8_t*)stackbuf, (size_t)n);
    return n;
  }

#if SERIAL_PRINTF_ALLOW_HEAP
  // Need a bigger buffer
  size_t need = (size_t)n + 1;
  buf = (char*)malloc(need);
  if (!buf)
  {
    // Fallback: truncated output
    g_serial_out->write((const uint8_t*)stackbuf, sizeof(stackbuf) - 1);
    return (int)(sizeof(stackbuf) - 1);
  }

  va_copy(ap2, ap);
  int n2 = vsnprintf(buf, need, fmt, ap2);
  va_end(ap2);

  if (n2 > 0) g_serial_out->write((const uint8_t*)buf, (size_t)n2);
  free(buf);
  return n2;
#else
  // Heap disabled: write truncated
  g_serial_out->write((const uint8_t*)stackbuf, sizeof(stackbuf) - 1);
  return (int)(sizeof(stackbuf) - 1);
#endif
}

int serial_printf(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int n = serial_vprintf(fmt, ap);
  va_end(ap);
  return n;
}
