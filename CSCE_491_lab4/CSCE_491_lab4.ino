#include <Arduino.h>
#include <math.h>

#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"

#if __has_include("array.h")
#include "array.h"
#define HAS_AUDIO_ARRAY 1
#else
#define HAS_AUDIO_ARRAY 0
#endif

// ============================================================================
// KEEP THE WORKING FILE'S BOARD MAPPING
// ============================================================================

#define AUDIO_PIN 33
#define LED_PIN   14

// ============================================================================
// General constants
// ============================================================================

#define NUM_LEDS                  100U
#define AUDIO_PWM_CHANNEL         0
#define AUDIO_PWM_RES_BITS        8U
#define AUDIO_PWM_FREQ            62500U
#define WS2812_RESET_US           60U
#define AUDIO_LED_UPDATE_INTERVAL 1024U

#if HAS_AUDIO_ARRAY
#define AUDIO_SAMPLE_RATE sampleRate
#else
#define AUDIO_SAMPLE_RATE 16000U
#endif

// ============================================================================
// Low-level RMT register addresses / bits
// ============================================================================

#define SYSTEM_PERIP_CLK_EN0_REG_ADDR   0x600C0018
#define SYSTEM_PERIP_RST_EN0_REG_ADDR   0x600C0020

#define RMT_CH0_CONF0_REG_ADDR          0x60016020
#define RMT_SYS_CONF_REG_ADDR           0x600160C0
#define RMT_INT_RAW_REG_ADDR            0x60016070
#define RMT_INT_CLR_REG_ADDR            0x6001607C
#define RMT_RAM_BASE_ADDR               0x60016800

#define SYSTEM_RMT_CLK_EN_BIT           (1U << 9)
#define SYSTEM_RMT_RST_BIT              (1U << 9)

#define RMT_APB_FIFO_MASK_BIT           (1U << 0)
#define RMT_MEM_CLK_FORCE_ON_BIT        (1U << 1)
#define RMT_SCLK_DIV_NUM_MASK           (0xFFU << 24)
#define RMT_CLK_EN_BIT                  (1UL << 31)

#define RMT_DIV_CNT_CH0_MASK            (0xFFU << 0)
#define RMT_TX_START_CH0_BIT            (1U << 9)
#define RMT_IDLE_OUT_LV_CH0_BIT         (1U << 12)
#define RMT_IDLE_OUT_EN_CH0_BIT         (1U << 13)
#define RMT_CARRIER_EN_CH0_BIT          (1U << 14)
#define RMT_CONF_UPDATE_CH0_BIT         (1U << 24)

#define RMT_CH0_TX_END_INT_RAW_BIT      (1U << 0)
#define RMT_CH0_TX_END_INT_CLR_BIT      (1U << 0)

// ============================================================================
// WS2812 timing in RMT clock cycles
// 80 MHz APB, divider = 1 => 12.5 ns per tick
// ============================================================================

#define WS2812_T1H_CYCLES               64U
#define WS2812_T1L_CYCLES               36U
#define WS2812_T0H_CYCLES               32U
#define WS2812_T0L_CYCLES               68U

// ============================================================================
// Globals
// ============================================================================

static uint32_t g_led_colors[NUM_LEDS];
static uint32_t g_sample_period_us = 0U;
static uint32_t g_sample_period_rem = 0U;

static float g_lowpass_120 = 0.0f;
static float g_lowpass_300 = 0.0f;
static float g_lowpass_600 = 0.0f;
static float g_lowpass_1200 = 0.0f;
static float g_lowpass_2400 = 0.0f;
static float g_band_envelopes[5] = {0.0f};

// ============================================================================
// Prototypes
// ============================================================================

void setup_RMT();
void transmit_led_signal(uint32_t *colors);

static void write_rmt_item(uint32_t index, uint16_t dur0, uint16_t lvl0,
                           uint16_t dur1, uint16_t lvl1);
static void encode_one_led_to_rmt_ram(uint32_t color);
static bool wait_for_rmt_done_and_clear();

static uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);
static uint32_t scale_color(uint32_t color, float scale);
static void clear_leds();
static void build_audio_visual_frame(uint8_t sample, uint32_t sample_index);

static void setup_audio_pwm();
static void update_audio_sample(uint8_t sample);
static void stop_audio_output();
static void play_audio_array_once();

// ============================================================================
// Setup / loop
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("BOOTED: entered setup()");

  g_sample_period_us = 1000000UL / AUDIO_SAMPLE_RATE;
  g_sample_period_rem = 1000000UL % AUDIO_SAMPLE_RATE;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(AUDIO_PIN, OUTPUT);
  digitalWrite(AUDIO_PIN, LOW);

  setup_audio_pwm();
  setup_RMT();

  Serial.println("TESTING LED...");
  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }
  g_led_colors[0] = make_color(255, 0, 0);
  transmit_led_signal(g_led_colors);
  delay(2000);

#if HAS_AUDIO_ARRAY
  Serial.println("AUDIO ARRAY: playing array.h once");
  play_audio_array_once();
  stop_audio_output();
#else
  Serial.println("AUDIO ARRAY: array.h not found, skipping playback");
#endif
}

void loop() {
  delay(1000);
}

// ============================================================================
// Audio path
// ============================================================================

static void setup_audio_pwm() {
  ledcAttachChannel(AUDIO_PIN, AUDIO_PWM_FREQ, AUDIO_PWM_RES_BITS, AUDIO_PWM_CHANNEL);
  ledcWrite(AUDIO_PIN, 0);
}

static void update_audio_sample(uint8_t sample) {
  ledcWrite(AUDIO_PIN, sample);
}

static void stop_audio_output() {
  ledcWrite(AUDIO_PIN, 0);
}

static void play_audio_array_once() {
#if HAS_AUDIO_ARRAY
  uint32_t sample_count = sizeof(sampleArray) / sizeof(sampleArray[0]);
  uint32_t next_sample_time = micros();
  uint32_t rem_accumulator = 0U;

  for (uint32_t i = 0U; i < sample_count; ++i) {
    uint8_t sample = sampleArray[i] & 0xFFU;
    update_audio_sample(sample);

    if ((i % AUDIO_LED_UPDATE_INTERVAL) == 0U) {
      build_audio_visual_frame(sample, i);
      transmit_led_signal(g_led_colors);
      yield();
    }

    next_sample_time += g_sample_period_us;
    rem_accumulator += g_sample_period_rem;
    if (rem_accumulator >= AUDIO_SAMPLE_RATE) {
      next_sample_time += 1U;
      rem_accumulator -= AUDIO_SAMPLE_RATE;
    }

    while ((int32_t)(micros() - next_sample_time) < 0) {
    }

    if ((i & 0x3FFU) == 0U) {
      yield();
    }
  }

  clear_leds();
#endif
}

// ============================================================================
// Low-level RMT setup / transmit for WS2812
// ============================================================================

void setup_RMT() {
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG_ADDR, SYSTEM_RMT_CLK_EN_BIT);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG_ADDR, SYSTEM_RMT_RST_BIT);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG_ADDR, SYSTEM_RMT_RST_BIT);

  uint32_t sys_conf = REG_READ(RMT_SYS_CONF_REG_ADDR);
  sys_conf &= ~RMT_APB_FIFO_MASK_BIT;
  sys_conf |= RMT_CLK_EN_BIT;
  sys_conf |= RMT_MEM_CLK_FORCE_ON_BIT;
  sys_conf &= ~RMT_SCLK_DIV_NUM_MASK;
  REG_WRITE(RMT_SYS_CONF_REG_ADDR, sys_conf);

  uint32_t ch0 = REG_READ(RMT_CH0_CONF0_REG_ADDR);
  ch0 &= ~RMT_CARRIER_EN_CH0_BIT;
  ch0 &= ~RMT_DIV_CNT_CH0_MASK;
  ch0 |= 1U;
  ch0 |= RMT_IDLE_OUT_EN_CH0_BIT;
  ch0 &= ~RMT_IDLE_OUT_LV_CH0_BIT;
  REG_WRITE(RMT_CH0_CONF0_REG_ADDR, ch0);

  REG_SET_BIT(RMT_CH0_CONF0_REG_ADDR, RMT_CONF_UPDATE_CH0_BIT);
  REG_WRITE(RMT_INT_CLR_REG_ADDR, RMT_CH0_TX_END_INT_CLR_BIT);

  // Route GPIO14 to RMT output and explicitly enable GPIO14 as an output
  PIN_FUNC_SELECT(IO_MUX_GPIO14_REG, PIN_FUNC_GPIO);
  REG_WRITE(GPIO_FUNC14_OUT_SEL_CFG_REG, 81);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, (1U << LED_PIN));
}

static void write_rmt_item(uint32_t index, uint16_t dur0, uint16_t lvl0,
                           uint16_t dur1, uint16_t lvl1) {
  volatile uint32_t *rmt_ram = (volatile uint32_t *)RMT_RAM_BASE_ADDR;

  uint32_t word = 0;
  word |= ((uint32_t)(dur0 & 0x7FFFU) << 0);
  word |= ((uint32_t)(lvl0 & 0x1U)    << 15);
  word |= ((uint32_t)(dur1 & 0x7FFFU) << 16);
  word |= ((uint32_t)(lvl1 & 0x1U)    << 31);

  rmt_ram[index] = word;
}

static void encode_one_led_to_rmt_ram(uint32_t color) {
  uint8_t g = (uint8_t)((color >> 8) & 0xFFU);
  uint8_t r = (uint8_t)((color >> 16) & 0xFFU);
  uint8_t b = (uint8_t)(color & 0xFFU);

  uint8_t bytes[3] = {g, r, b};

  uint32_t idx = 0U;
  for (uint32_t byte_i = 0U; byte_i < 3U; ++byte_i) {
    for (int bit = 7; bit >= 0; --bit) {
      bool one = (((bytes[byte_i] >> bit) & 0x01U) != 0U);

      if (one) {
        write_rmt_item(idx, WS2812_T1H_CYCLES, 1U, WS2812_T1L_CYCLES, 0U);
      } else {
        write_rmt_item(idx, WS2812_T0H_CYCLES, 1U, WS2812_T0L_CYCLES, 0U);
      }
      ++idx;
    }
  }

  write_rmt_item(24U, 0U, 0U, 0U, 0U);
}

static bool wait_for_rmt_done_and_clear() {
  uint32_t start = micros();

  while ((REG_READ(RMT_INT_RAW_REG_ADDR) & RMT_CH0_TX_END_INT_RAW_BIT) == 0U) {
    if ((uint32_t)(micros() - start) > 2000U) {
      Serial.println("RMT timeout");
      return false;
    }
    yield();
  }

  REG_WRITE(RMT_INT_CLR_REG_ADDR, RMT_CH0_TX_END_INT_CLR_BIT);
  return true;
}

void transmit_led_signal(uint32_t *colors) {
  for (uint32_t led = 0U; led < NUM_LEDS; ++led) {
    encode_one_led_to_rmt_ram(colors[led]);

    REG_SET_BIT(RMT_CH0_CONF0_REG_ADDR, RMT_CONF_UPDATE_CH0_BIT);
    REG_SET_BIT(RMT_CH0_CONF0_REG_ADDR, RMT_TX_START_CH0_BIT);

    if (!wait_for_rmt_done_and_clear()) {
      return;
    }

    if ((led & 0x07U) == 0U) {
      yield();
    }
  }

  delayMicroseconds(WS2812_RESET_US);
}

// ============================================================================
// LED helpers / visualizer
// ============================================================================

static uint32_t make_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static uint32_t scale_color(uint32_t color, float scale) {
  if (scale < 0.0f) scale = 0.0f;
  if (scale > 1.0f) scale = 1.0f;

  uint8_t r = (uint8_t)(((color >> 16) & 0xFFU) * scale);
  uint8_t g = (uint8_t)(((color >> 8) & 0xFFU) * scale);
  uint8_t b = (uint8_t)((color & 0xFFU) * scale);
  return make_color(r, g, b);
}

static void clear_leds() {
  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }
  transmit_led_signal(g_led_colors);
}

static void build_audio_visual_frame(uint8_t sample, uint32_t sample_index) {
  const uint32_t NUM_BANDS = 5U;
  const uint32_t LEDS_PER_BAND = NUM_LEDS / NUM_BANDS;

  const float x = ((float)sample - 128.0f) / 128.0f;
  const uint32_t band_colors[NUM_BANDS] = {
    make_color(255, 0, 0),
    make_color(255, 128, 0),
    make_color(0, 0, 255),
    make_color(0, 255, 0),
    make_color(255, 0, 255)
  };

  g_lowpass_120  += 0.045f * (x - g_lowpass_120);
  g_lowpass_300  += 0.10f  * (x - g_lowpass_300);
  g_lowpass_600  += 0.18f  * (x - g_lowpass_600);
  g_lowpass_1200 += 0.33f  * (x - g_lowpass_1200);
  g_lowpass_2400 += 0.55f  * (x - g_lowpass_2400);

  const float band_levels[NUM_BANDS] = {
    fabsf(g_lowpass_120),
    fabsf(g_lowpass_300 - g_lowpass_120),
    fabsf(g_lowpass_600 - g_lowpass_300),
    fabsf(g_lowpass_1200 - g_lowpass_600),
    fabsf(g_lowpass_2400 - g_lowpass_1200)
  };

  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }

  for (uint32_t band = 0U; band < NUM_BANDS; ++band) {
    float target = band_levels[band] * 2.4f;
    if (target > 1.0f) target = 1.0f;

    float response = (target > g_band_envelopes[band]) ? 0.50f : 0.15f;
    g_band_envelopes[band] += response * (target - g_band_envelopes[band]);

    uint32_t lit_count = (uint32_t)(g_band_envelopes[band] * LEDS_PER_BAND + 0.5f);
    uint32_t start = band * LEDS_PER_BAND;
    uint32_t end = start + LEDS_PER_BAND;
    uint32_t peak = start + ((sample_index / AUDIO_LED_UPDATE_INTERVAL) % LEDS_PER_BAND);

    for (uint32_t led = start; led < end; ++led) {
      uint32_t color = 0U;
      uint32_t relative = led - start;

      if (relative < lit_count) {
        float fade = 1.0f - ((float)relative / (float)LEDS_PER_BAND);
        color = scale_color(band_colors[band], 0.18f + 0.42f * fade);
      }

      if ((led == peak) && (g_band_envelopes[band] > 0.08f)) {
        color = scale_color(band_colors[band], 0.72f);
      }

      g_led_colors[led] = color;
    }
  }
}
