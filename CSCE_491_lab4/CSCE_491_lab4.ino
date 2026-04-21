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

#define AUDIO_PIN 33
#define LED_PIN   14


#define NUM_LEDS                  100U
#define AUDIO_PWM_CHANNEL         0
#define AUDIO_PWM_RES_BITS        8U
#define AUDIO_PWM_FREQ            62500U
#define WS2812_RESET_US           60U
#define AUDIO_LED_UPDATE_INTERVAL 1024U

// If array.h exists, use the sample rate defined there.
// Otherwise default to 16000 Hz.
#if HAS_AUDIO_ARRAY
#define AUDIO_SAMPLE_RATE sampleRate
#else
#define AUDIO_SAMPLE_RATE 16000U
#endif


// System clock enable/reset registers for peripherals
#define SYSTEM_PERIP_CLK_EN0_REG_ADDR   0x600C0018
#define SYSTEM_PERIP_RST_EN0_REG_ADDR   0x600C0020

// RMT registers
#define RMT_CH0_CONF0_REG_ADDR          0x60016020
#define RMT_SYS_CONF_REG_ADDR           0x600160C0
#define RMT_INT_RAW_REG_ADDR            0x60016070
#define RMT_INT_CLR_REG_ADDR            0x6001607C
#define RMT_RAM_BASE_ADDR               0x60016800

// Bit masks for enabling/resetting RMT
#define SYSTEM_RMT_CLK_EN_BIT           (1U << 9)
#define SYSTEM_RMT_RST_BIT              (1U << 9)

// Misc RMT config bits
#define RMT_APB_FIFO_MASK_BIT           (1U << 0)
#define RMT_MEM_CLK_FORCE_ON_BIT        (1U << 1)
#define RMT_SCLK_DIV_NUM_MASK           (0xFFU << 24)
#define RMT_CLK_EN_BIT                  (1UL << 31)

// Channel 0 config bits
#define RMT_DIV_CNT_CH0_MASK            (0xFFU << 0)
#define RMT_TX_START_CH0_BIT            (1U << 9)
#define RMT_IDLE_OUT_LV_CH0_BIT         (1U << 12)
#define RMT_IDLE_OUT_EN_CH0_BIT         (1U << 13)
#define RMT_CARRIER_EN_CH0_BIT          (1U << 14)
#define RMT_CONF_UPDATE_CH0_BIT         (1U << 24)

// Interrupt bits
#define RMT_CH0_TX_END_INT_RAW_BIT      (1U << 0)
#define RMT_CH0_TX_END_INT_CLR_BIT      (1U << 0)


#define WS2812_T1H_CYCLES               64U
#define WS2812_T1L_CYCLES               36U
#define WS2812_T0H_CYCLES               32U
#define WS2812_T0L_CYCLES               68U

// Array holding the 24-bit color for each LED
static uint32_t g_led_colors[NUM_LEDS];

// Integer part of the sample period in microseconds
static uint32_t g_sample_period_us = 0U;

// Remainder part of the sample period
static uint32_t g_sample_period_rem = 0U;

//low-pass filter states used to create fake frequency bands for LED
static float g_lowpass_120 = 0.0f;
static float g_lowpass_300 = 0.0f;
static float g_lowpass_600 = 0.0f;
static float g_lowpass_1200 = 0.0f;
static float g_lowpass_2400 = 0.0f;
static float g_band_envelopes[5] = {0.0f};

void setup_RMT();

// Sends one full LED frame to the strip
void transmit_led_signal(uint32_t *colors);

//one RMT item into RMT memory
static void write_rmt_item(uint32_t index, uint16_t dur0, uint16_t lvl0,
                           uint16_t dur1, uint16_t lvl1);

// Converts one LED color timing data in RMT RAM
static void encode_one_led_to_rmt_ram(uint32_t color);

static bool wait_for_rmt_done_and_clear();

//pack RGB into a 32-bit integer
static uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);

//brightness RGB color
static uint32_t scale_color(uint32_t color, float scale);

// Turns all LEDs off
static void clear_leds();

// Builds one LED visualizer frame from an audio sample
static void build_audio_visual_frame(uint8_t sample, uint32_t sample_index);

// Audio PWM setup and control
static void setup_audio_pwm();
static void update_audio_sample(uint8_t sample);
static void stop_audio_output();
static void play_audio_array_once();


void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("BOOTED: entered setup()");

// timing for audio playback
  g_sample_period_us = 1000000UL / AUDIO_SAMPLE_RATE;
  g_sample_period_rem = 1000000UL % AUDIO_SAMPLE_RATE;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(AUDIO_PIN, OUTPUT);
  digitalWrite(AUDIO_PIN, LOW);

  // Initialize audio PWM output hardware
  setup_audio_pwm();

  // Initialize RMT hardware for the LED strip
  setup_RMT();

  Serial.println("TESTING LED...");

  // Clear all LED colors
  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }

  // Turn only the first LED red
  g_led_colors[0] = make_color(255, 0, 0);

  // Send that frame to the strip
  transmit_led_signal(g_led_colors);

  // Leave it for 2 seconds
  delay(2000);

#if HAS_AUDIO_ARRAY
  // If array.h exists, play the audio once
  Serial.println("AUDIO ARRAY: playing array.h once");
  play_audio_array_once();

  // Stop PWM after playback is done
  stop_audio_output();
#else
  // If array.h is missing, tell me and skip play
  Serial.println("AUDIO ARRAY: array.h not found, skipping playback");
#endif
}

void loop() {
  delay(1000);
}

static void setup_audio_pwm() {
  // Attach the audio pin to the chosen LEDC PWM channel
  ledcAttachChannel(AUDIO_PIN, AUDIO_PWM_FREQ, AUDIO_PWM_RES_BITS, AUDIO_PWM_CHANNEL);
  ledcWrite(AUDIO_PIN, 0);
}

static void update_audio_sample(uint8_t sample) {
  // Output the current 8-bit sample value on the PWM channel
  ledcWrite(AUDIO_PIN, sample);
}

static void stop_audio_output() {
  ledcWrite(AUDIO_PIN, 0);
}

static void play_audio_array_once() {
#if HAS_AUDIO_ARRAY
  uint32_t sample_count = sizeof(sampleArray) / sizeof(sampleArray[0]);

  // Track the exact time the next sample should be played
  uint32_t next_sample_time = micros();

  //fractional timing error
  uint32_t rem_accumulator = 0U;

  for (uint32_t i = 0U; i < sample_count; ++i) {
    // Grab the current sample and keep only the lower 8 bits
    uint8_t sample = sampleArray[i] & 0xFFU;

    // Output this sample through PWM
    update_audio_sample(sample);

    // update the LEDs so they react to the audio
    if ((i % AUDIO_LED_UPDATE_INTERVAL) == 0U) {
      build_audio_visual_frame(sample, i);
      transmit_led_signal(g_led_colors);
      yield();
    }

    // Advance by the integer portion of the sample period
    next_sample_time += g_sample_period_us;
    rem_accumulator += g_sample_period_rem;

    // Once enough remainder builds up, add one extra microsecond
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


void setup_RMT() {
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG_ADDR, SYSTEM_RMT_CLK_EN_BIT);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG_ADDR, SYSTEM_RMT_RST_BIT);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG_ADDR, SYSTEM_RMT_RST_BIT);

  uint32_t sys_conf = REG_READ(RMT_SYS_CONF_REG_ADDR);

  // Disable APB FIFO mask behavior
  sys_conf &= ~RMT_APB_FIFO_MASK_BIT;

  // Enable RMT clock
  sys_conf |= RMT_CLK_EN_BIT;
  sys_conf |= RMT_MEM_CLK_FORCE_ON_BIT;

  // Clear clock divider bits
  sys_conf &= ~RMT_SCLK_DIV_NUM_MASK;

  // Write updated system config back
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
  PIN_FUNC_SELECT(IO_MUX_GPIO14_REG, PIN_FUNC_GPIO);
  REG_WRITE(GPIO_FUNC14_OUT_SEL_CFG_REG, 81);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, (1U << LED_PIN));
}

static void write_rmt_item(uint32_t index, uint16_t dur0, uint16_t lvl0,
                           uint16_t dur1, uint16_t lvl1) {
  // Pointer to the base of RMT RAM
  volatile uint32_t *rmt_ram = (volatile uint32_t *)RMT_RAM_BASE_ADDR;
  uint32_t word = 0;

  word |= ((uint32_t)(dur0 & 0x7FFFU) << 0);
  word |= ((uint32_t)(lvl0 & 0x1U)    << 15);
  word |= ((uint32_t)(dur1 & 0x7FFFU) << 16);
  word |= ((uint32_t)(lvl1 & 0x1U)    << 31);

  // Store the encoded item into RMT RAM
  rmt_ram[index] = word;
}

static void encode_one_led_to_rmt_ram(uint32_t color) {
  // WS2812 expects color order as G, R, B
  uint8_t g = (uint8_t)((color >> 8) & 0xFFU);
  uint8_t r = (uint8_t)((color >> 16) & 0xFFU);
  uint8_t b = (uint8_t)(color & 0xFFU);

  // Store the three bytes in the exact order the LED wants
  uint8_t bytes[3] = {g, r, b};

  // Index into RMT memory
  uint32_t idx = 0U;

  for (uint32_t byte_i = 0U; byte_i < 3U; ++byte_i) {
    // For each byte, send bits from MSB to LSB
    for (int bit = 7; bit >= 0; --bit) {
      // Check whether the current bit is a 1 or 0
      bool one = (((bytes[byte_i] >> bit) & 0x01U) != 0U);

      // Encode timing based on whether the bit is a 1 or 0
      if (one) {
        write_rmt_item(idx, WS2812_T1H_CYCLES, 1U, WS2812_T1L_CYCLES, 0U);
      } else {
        write_rmt_item(idx, WS2812_T0H_CYCLES, 1U, WS2812_T0L_CYCLES, 0U);
      }

      // Move to the next RMT memory slot
      ++idx;
    }
  }

  write_rmt_item(24U, 0U, 0U, 0U, 0U);
}

static bool wait_for_rmt_done_and_clear() {
  // Record when we started waiting
  uint32_t start = micros();

  // Wait until the TX done interrupt flag becomes set
  while ((REG_READ(RMT_INT_RAW_REG_ADDR) & RMT_CH0_TX_END_INT_RAW_BIT) == 0U) {
    // Timeout after 2000 us to avoid hanging forever
    if ((uint32_t)(micros() - start) > 2000U) {
      Serial.println("RMT timeout");
      return false;
    }

    // Yield to keep system alive
    yield();
  }

  // Clear the TX done interrupt flag
  REG_WRITE(RMT_INT_CLR_REG_ADDR, RMT_CH0_TX_END_INT_CLR_BIT);

  return true;
}

void transmit_led_signal(uint32_t *colors) {
  // Send one LED at a time
  for (uint32_t led = 0U; led < NUM_LEDS; ++led) {
    // Convert this LED's color into RMT pulse data
    encode_one_led_to_rmt_ram(colors[led]);

    // Tell RMT to update config and start transmitting
    REG_SET_BIT(RMT_CH0_CONF0_REG_ADDR, RMT_CONF_UPDATE_CH0_BIT);
    REG_SET_BIT(RMT_CH0_CONF0_REG_ADDR, RMT_TX_START_CH0_BIT);

    // Wait until this LED's transmission finishes
    if (!wait_for_rmt_done_and_clear()) {
      return;
    }

    if ((led & 0x07U) == 0U) {
      yield();
    }
  }

  // Final reset/latch period so the strip accepts the new frame
  delayMicroseconds(WS2812_RESET_US);
}

static uint32_t make_color(uint8_t r, uint8_t g, uint8_t b) {
  // Pack R, G, and B into one 32-bit integer
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static uint32_t scale_color(uint32_t color, float scale) {
  if (scale < 0.0f) scale = 0.0f;
  if (scale > 1.0f) scale = 1.0f;

  // Extract each color component, scale it, then rebuild the color
  uint8_t r = (uint8_t)(((color >> 16) & 0xFFU) * scale);
  uint8_t g = (uint8_t)(((color >> 8) & 0xFFU) * scale);
  uint8_t b = (uint8_t)((color & 0xFFU) * scale);

  return make_color(r, g, b);
}

static void clear_leds() {
  // Set all stored colors to off
  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }

  // Send the cleared frame to the strip
  transmit_led_signal(g_led_colors);
}

static void build_audio_visual_frame(uint8_t sample, uint32_t sample_index) {
  const uint32_t NUM_BANDS = 5U;
  const uint32_t LEDS_PER_BAND = NUM_LEDS / NUM_BANDS;
  const float x = ((float)sample - 128.0f) / 128.0f;

  // Color assigned to each band
  const uint32_t band_colors[NUM_BANDS] = {
    make_color(255, 0, 0),     // red
    make_color(255, 128, 0),   // orange
    make_color(0, 0, 255),     // blue
    make_color(0, 255, 0),     // green
    make_color(255, 0, 255)    // magenta
  };

  g_lowpass_120  += 0.045f * (x - g_lowpass_120);
  g_lowpass_300  += 0.10f  * (x - g_lowpass_300);
  g_lowpass_600  += 0.18f  * (x - g_lowpass_600);
  g_lowpass_1200 += 0.33f  * (x - g_lowpass_1200);
  g_lowpass_2400 += 0.55f  * (x - g_lowpass_2400);

  // Create rough frequency bands by subtracting adjacent low-pass outputs
  const float band_levels[NUM_BANDS] = {
    fabsf(g_lowpass_120),
    fabsf(g_lowpass_300 - g_lowpass_120),
    fabsf(g_lowpass_600 - g_lowpass_300),
    fabsf(g_lowpass_1200 - g_lowpass_600),
    fabsf(g_lowpass_2400 - g_lowpass_1200)
  };

  // Clear all LEDs before rebuilding the new frame
  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }

  for (uint32_t band = 0U; band < NUM_BANDS; ++band) {
    // Scale the band level upward a little
    float target = band_levels[band] * 2.4f;

    // Clamp to at most 1.0
    if (target > 1.0f) target = 1.0f;

    // Attack faster when rising, decay slower when falling
    float response = (target > g_band_envelopes[band]) ? 0.50f : 0.15f;
    g_band_envelopes[band] += response * (target - g_band_envelopes[band]);

    // Determine how many LEDs in this band should be lit
    uint32_t lit_count = (uint32_t)(g_band_envelopes[band] * LEDS_PER_BAND + 0.5f);

    // LED index range for this band
    uint32_t start = band * LEDS_PER_BAND;
    uint32_t end = start + LEDS_PER_BAND;
    uint32_t peak = start + ((sample_index / AUDIO_LED_UPDATE_INTERVAL) % LEDS_PER_BAND);

    // Fill this band's LEDs
    for (uint32_t led = start; led < end; ++led) {
      uint32_t color = 0U;
      uint32_t relative = led - start;

      // Light up LEDs up to lit_count with a fade effect
      if (relative < lit_count) {
        float fade = 1.0f - ((float)relative / (float)LEDS_PER_BAND);
        color = scale_color(band_colors[band], 0.18f + 0.42f * fade);
      }

      // Draw a brighter moving peak indicator if the band is active enough
      if ((led == peak) && (g_band_envelopes[band] > 0.08f)) {
        color = scale_color(band_colors[band], 0.72f);
      }

      g_led_colors[led] = color;
    }
  }
}