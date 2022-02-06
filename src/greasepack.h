#pragma once
#include <stddef.h>
#include <stdint.h>

// Encoding flux of duration T:
// 0: Impossible
// 1..249: Encodes as 1 byte (T itself)
// 250..1524: Encodes as 2 bytes: (T // 255 + 250), (
// 1525..2^28: Encodes as 6 bytes:  255 + Space + "write_28bit"
enum { cutoff_1byte = 250, cutoff_2byte = 1525, cutoff_6byte = (1 << 28) - 1 };

// Pack one flux duration into greaseaweazel format.
// buf: Pointer to the current location in the flux buffer. NULL indicates no
// buffer, regardless of end. end: Pointer to the end of the flux buffer value:
// the flux value itself
//
// Returns: the new 'buf'.  If buf==end, then the buffer is now full, and
// the last byte is a terminating 0. This can also mean that the last value
// was not stored because there was insufficient space, but there's no way to
// tell apart an "exactly full" buffer from "the last sample didn't fit".
static inline uint8_t *greasepack(uint8_t *buf, uint8_t *end, unsigned value) {
  // already no space left
  if (!buf || buf == end) {
    return buf;
  }

  size_t left = end - buf;
  size_t need = value < cutoff_1byte ? 1 : value < cutoff_2byte ? 2 : 6;

  // Buffer's going to be too full, store a terminating 0 and give up
  if (need > left) {
    *buf = 0;
    return end;
  }

  if (value < cutoff_1byte) {
    *buf++ = value;
  } else if (value < cutoff_2byte) {
    unsigned high = (value - 250) / 255;
    *buf++ = 250 + high;
    *buf++ = 1 + (value - 250) % 255;
  } else {
    if (value > cutoff_6byte) {
      value = cutoff_6byte;
    }
    *buf++ = 255;
    *buf++ = 2;
    *buf++ = 1 | (value << 1) & 255;
    *buf++ = 1 | (value >> 6) & 255;
    *buf++ = 1 | (value >> 13) & 255;
    *buf++ = 1 | (value >> 20) & 255;
  }

  return buf;
}