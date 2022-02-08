#if defined(PICO_BOARD) || defined(__RP2040__) || defined(ARDUINO_ARCH_RP2040)
#include "greasepack.h"
#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const int fluxread_sideset_pin_count = 0;
static const bool fluxread_sideset_enable = 0;
static const uint16_t fluxread[] = {
    // ; Count flux pulses and watch for index pin
    // ; flux input is the 'jmp pin'.  index is "pin zero".
    // ; Counts are in units 3 / F_pio, so e.g., at 30MHz 1 count = 0.1us
    // ; Count down while waiting for the counter to go HIGH
    // ; The only counting is down, so C code will just have to negate the
    // count!
    // ; Each 'wait one' loop takes 3 instruction-times
    // wait_one:
    0x0041, //     jmp x--, wait_one_next ; acts as a non-conditional decrement
            //     of x
            // wait_one_next:
    0x00c3, //     jmp pin wait_zero
    0x0000, //     jmp wait_one
            // ; Each 'wait zero' loop takes 3 instruction-times, needing one
            // instruction delay
            // ; (it has to match the 'wait one' timing exactly)
            // wait_zero:
    0x0044, //     jmp x--, wait_zero_next ; acts as a non-conditional decrement
            //     of x
            // wait_zero_next:
    0x01c3, //     jmp pin wait_zero [1]
            // ; Top bit is index status, bottom 15 bits are inverse of counts
            // ; Combined FIFO gives 16 entries (8 32-bit entries) so with the
    // ; smallest plausible pulse of 2us there are 250 CPU cycles available
    // @125MHz
    0x4001, //     in pins, 1
    0x402f, //     in x, 15
    // ; Threee cycles for the end of loop, so we need to decrement x to make
    // everything
    // ; come out right. This has constant timing whether we actually jump back
    // vs wrapping.
    0x0040, //     jmp x--, wait_one
};
static const pio_program_t fluxread_struct = {.instructions = fluxread,
                                 .length =
                                     sizeof(fluxread) / sizeof(fluxread[0]),
                                 .origin = -1};

static const int fluxwrite_sideset_pin_count = 1;
static const bool fluxwrite_sideset_enable = 1;
static const uint16_t fluxwrite[] = {
            // .side_set 1 opt
            // start:
            // ;; ensure wgate is deasserted then wait for FIFO to be loaded
    0xb842, //     nop side 1
    0x80e0, //     pull ifempty
            // ; Wait for a falling edge on the index pin
            // wait_index_high:
    0x00c4, //     jmp pin wait_index_low
    0x0002, //     jmp wait_index_high
            // wait_index_low:
    0x00c4, //     jmp pin wait_index_low
            // ; Enable gate 2us before writing
    0xb042, //     nop side 0
    0xe033, //     set x 19
            // loop_gate:
    0x0147, //     jmp x--, loop_gate [1]
            // loop_flux:
    0xe000, //     set pins, 0 ; drive pin low
    0x6030, //     out x, 16  ; get the next timing pulse information
            // ;; If x is zero here, either a 0 was explicitly put into the flux timing stream,
            // ;; OR the data ended naturally OR there was an under-run. In any case, jump back
            // ;; to the beginning to wait for available data and then an index pulse -- we're done.
    0x0020, //     jmp !x, start
            // ;; output the fixed on time.  10 cycles (preload reg with 6) is about 0.5us.
            // ;; note that wdc1772 has varying low times, from 570 to 1380us
    0xe046, //     set y, 6 ; fixed on time
            // loop_low:
    0x008c, //     jmp y--, loop_low
    0xe001, //     set pins, 1 ; drive pin high
    0x80c0, //     pull ifempty noblock
            // loop_high:
    0x004f, //     jmp x--, loop_high
    0x0008, //     jmp loop_flux
};
static const pio_program_t fluxwrite_struct = {.instructions = fluxwrite,
                                  .length =
                                      sizeof(fluxwrite) / sizeof(fluxwrite[0]),
                                  .origin = -1};

typedef struct floppy_singleton {
  PIO pio;
  unsigned sm;
  uint16_t offset;
  uint16_t half;
} floppy_singleton_t;

static floppy_singleton_t g_reader, g_writer;

const static PIO pio_instances[2] = {pio0, pio1};
static bool allocate_pio_set_program(floppy_singleton_t *info, const pio_program_t *program) {
  memset(info, 0, sizeof(*info));
  for (size_t i = 0; i < NUM_PIOS; i++) {
    PIO pio = pio_instances[i];
    if (!pio_can_add_program(pio, program)) {
      continue;
    }
    int sm = pio_claim_unused_sm(pio, false);
    if (sm != -1) {
      info->pio = pio;
      info->sm = sm;
      // cannot fail, we asked nicely already
      info->offset = pio_add_program(pio, program);
      return true;
    }
  }
  return false;
}

static bool init_capture(int index_pin, int read_pin) {
  if (g_reader.pio) {
    return true;
  }

  if (!allocate_pio_set_program(&g_reader, &fluxread_struct)) {
    return false;
  }

  gpio_pull_up(index_pin);

  pio_sm_config c = {0, 0, 0};
  sm_config_set_wrap(&c, g_reader.offset,
                     g_reader.offset + fluxread_struct.length - 1);
  sm_config_set_jmp_pin(&c, read_pin);
  sm_config_set_in_pins(&c, index_pin);
  sm_config_set_in_shift(&c, true, true, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, 1 << read_pin,
                            1 << read_pin);
  float div = (float)clock_get_hz(clk_sys) / (3 * 24e6);
  sm_config_set_clkdiv(&c, div); // 72MHz capture clock / 24MHz sample rate

  pio_sm_init(g_reader.pio, g_reader.sm, g_reader.offset, &c);

  // g_reader.dreq = pio_get_dreq(g_reader.pio, g_reader.sm, false);
  // g_reader.dma = dma_claim_unused_channel(false);
  // rx_source = g_reader.pio->rxf[g_reader.sm];
  // dma_channel_configure(g_reader.dma, &c, tx_destination,
  return true;
}

static void start_common() {
  pio_sm_exec(g_reader.pio, g_reader.sm, g_reader.offset);
  pio_sm_restart(g_reader.pio, g_reader.sm);
}

static uint16_t read_fifo() {
  if (g_reader.half) {
    uint16_t result = g_reader.half;
    g_reader.half = 0;
    return result;
  }
  uint32_t value = pio_sm_get_blocking(g_reader.pio, g_reader.sm);
  g_reader.half = value >> 16;
  return value & 0xffff;
}

static void disable_capture(void) {
  pio_sm_set_enabled(g_reader.pio, g_reader.sm, false);
}

static void free_capture(void) {
  if (!g_reader.pio) {
    // already deinit
    return;
  }
  disable_capture();
  pio_sm_unclaim(g_reader.pio, g_reader.sm);
  pio_remove_program(g_reader.pio, &fluxwrite_struct, g_reader.offset);
  memset(&g_reader, 0, sizeof(g_reader));
}

static void capture_foreground(int index_pin, uint8_t *start, uint8_t *end,
                               bool wait_index, bool stop_index,
                               uint32_t *falling_index_offset,
                               bool store_greaseweazle) {
  if (falling_index_offset) {
    *falling_index_offset = ~0u;
  }
  start_common();

  // wait for a falling edge of index pin, then enable the capture peripheral
  if (wait_index) {
    while (!gpio_get(index_pin)) { /* NOTHING */
    }
    while (gpio_get(index_pin)) { /* NOTHING */
    }
  }

  pio_sm_set_enabled(g_reader.pio, g_reader.sm, true);
  int last = read_fifo();
  int i = 0;
  while (start != end) {
    int data = read_fifo();
    int delta = last - data;
    if (delta < 0)
      delta += 65536;
    delta /= 2;
    if (!(data & 1) && (last & 1)) {
      if (falling_index_offset) {
        *falling_index_offset = i;
        falling_index_offset = NULL;
        if (stop_index)
          break;
      }
    }
    i++;
    last = data;
    if (store_greaseweazle) {
      start = greasepack(start, end, delta);
    } else {
      *start++ = delta > 255 ? 255 : delta;
    }
  }

  disable_capture();
}

static void enable_capture_fifo() { start_common(); }

static bool init_write(int index_pin, int wrgate_pin, int wrdata_pin) {
  if (g_reader.pio) {
    return true;
  }

  if (!allocate_pio_set_program(&g_reader, &fluxread_struct)) {
    return false;
  }

  uint32_t wrgate_bit = 1u << wrgate_pin;
  uint32_t wrdata_bit = 1u << wrdata_pin;
  uint32_t index_bit = 1u << index_pin;
  pio_sm_set_pindirs_with_mask(g_reader.pio, g_reader.sm, wrgate_bit | wrdata_bit, wrgate_bit | wrdata_bit | index_bit);
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, wrgate_bit | wrdata_bit | index_bit, wrgate_bit | wrdata_bit | index_bit);
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, 0, wrgate_bit | wrdata_bit | index_bit);
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, wrgate_bit | wrdata_bit | index_bit, wrgate_bit | wrdata_bit | index_bit);
  pio_gpio_init(g_reader.pio, wrgate_pin);
  pio_gpio_init(g_reader.pio, wrdata_pin);
  gpio_pull_up(index_pin);

  pio_sm_config c = {0, 0, 0, 0};
  sm_config_set_wrap(&c, g_writer.offset,
                     g_writer.offset + fluxread_struct.length - 1);
  sm_config_set_jmp_pin(&c, index_pin);
  sm_config_set_set_pins(&c, wrdata_pin, 1);
  sm_config_set_sideset_pins(&c, wrgate_pin);
  sm_config_set_sideset(&c, fluxwrite_sideset_pin_count, fluxwrite_sideset_enable, false);
  sm_config_set_out_shift(&c, true, true, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  float div = (float)clock_get_hz(clk_sys) / (24e6);
  sm_config_set_clkdiv(&c, div); // 24MHz output clock

  pio_sm_init(g_writer.pio, g_writer.sm, g_writer.offset, &c);


  return true;
}

#define OVERHEAD (15) // minimum pulse length due to PIO overhead, about 0.625us
static void write_fifo(unsigned value) {
  if (value < OVERHEAD) { value = 1; }
  else { value -= OVERHEAD; if(value > 0xffff) value = 0xffff; }
  if (g_writer.half) {
    unsigned write_val = (g_writer.half << 16) | value;
    pio_sm_put_blocking(g_writer.pio, g_writer.sm, write_val);
    g_writer.half = 0;
  } else {
    g_writer.half = value;
  }
}

static void disable_write() {
    pio_sm_exec(g_writer.pio, g_writer.sm, g_writer.offset);
    pio_sm_restart(g_writer.pio, g_writer.sm);
}

static void write_foreground(int index_pin, uint8_t *pulses, uint8_t *pulse_end, bool store_greaseweazle) {
  int old_index_state = gpio_get(index_pin);
  int index_count = 0;

  pio_sm_exec(g_writer.pio, g_writer.sm, g_writer.offset);
  pio_sm_restart(g_writer.pio, g_writer.sm);

  Serial.printf("write_foreground pulses=%p pulse_end=%p\n", pulses, pulse_end);
  while(pulses != pulse_end) {
    unsigned value = greaseunpack(&pulses, pulse_end, store_greaseweazle);
    write_fifo(value);
    do {
      int index_state = gpio_get(index_pin);
      if(old_index_state && !index_state) {
        index_count ++; 
        // A full revolution has occurred, stop writing immediately
        if(index_count == 2) {
            disable_write();
            Serial.printf("disable_write pulses=%p pulse_end=%p\n", pulses, pulse_end);
            break;
        }
      }
    } while(pio_sm_is_tx_fifo_full(g_writer.pio, g_writer.sm));
  }
  Serial.printf("end of write pulses=%p pulse_end=%p\n", pulses, pulse_end);
}

static void free_write() {
  if (!g_writer.pio) {
    // already deinit
    return;
  }
  disable_write();
  pio_sm_unclaim(g_writer.pio, g_writer.sm);
  pio_remove_program(g_writer.pio, &fluxread_struct, g_writer.offset);
  memset(&g_writer, 0, sizeof(g_writer));
}

#ifdef __cplusplus
#include <Adafruit_Floppy.h>

uint32_t rp2040_flux_capture(int index_pin, int rdpin, volatile uint8_t *pulses,
                             volatile uint8_t *pulse_end,
                             uint32_t *falling_index_offset,
                             bool store_greaseweazle) {
  init_capture(index_pin, rdpin);
  capture_foreground(index_pin, (uint8_t *)pulses, (uint8_t *)pulse_end, true,
                     false, falling_index_offset, store_greaseweazle);
  free_capture();
  return pulse_end - pulses;
}

unsigned _last = ~0u;
bool Adafruit_Floppy::init_capture(void) {
  _last = ~0u;
  return ::init_capture(_indexpin, _rddatapin);
}

bool Adafruit_Floppy::start_polled_capture(void) {
  if (!init_capture())
    return false;
  start_common();
  pio_sm_set_enabled(g_reader.pio, g_reader.sm, true);
  return true;
}

void Adafruit_Floppy::disable_capture(void) { ::disable_capture(); }

uint16_t mfm_io_sample_flux(bool *index) {
  if (_last == ~0u) {
    _last = read_fifo();
  }
  int data = read_fifo();
  int delta = _last - data;
  _last = data;
  if (delta < 0)
    delta += 65536;
  *index = data & 1;
  return delta / 2;
}

void rp2040_flux_write(int index_pin, int wrgate_pin, int wrdata_pin, uint8_t *pulses, uint8_t *pulse_end, bool store_greaseweazle) {
  if (!init_write(index_pin, wrgate_pin, wrdata_pin)) { return; }
  write_foreground(index_pin, (uint8_t *)pulses, (uint8_t *)pulse_end, store_greaseweazle);
  uint32_t wrgate_bit = 1u << wrgate_pin;
  uint32_t wrdata_bit = 1u << wrdata_pin;
  uint32_t index_bit = 1u << index_pin;
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, wrgate_bit | wrdata_bit | index_bit, wrgate_bit | wrdata_bit | index_bit);
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, 0, wrgate_bit | wrdata_bit | index_bit);
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, wrgate_bit | wrdata_bit | index_bit, wrgate_bit | wrdata_bit | index_bit);
  free_write();
}

#endif
#endif
