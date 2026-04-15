#include <Arduino.h>
#include <math.h>
#include "soc/soc.h"   // gives REG_READ / REG_WRITE / REG_SET_BIT / REG_CLR_BIT

#if __has_include("array.h")
#include "array.h"
#define HAS_AUDIO_ARRAY 1
#else
#define HAS_AUDIO_ARRAY 0
#endif


//   GPIO14 -> LEDC
//   GPIO33 -> RMT
#define AUDIO_PIN 14
#define LED_PIN   33

#define NUM_LEDS 100U
#define PWM_RES_BITS 8U
#define SPEAKER_TEST_HZ 880.0f
#define SPEAKER_TEST_MS 250U
#define COLOR_STEPS 96U
#define COLOR_STEP_DELAY_MS 20U
#define WS2812_RESET_US 60U

#if HAS_AUDIO_ARRAY
#define AUDIO_SAMPLE_RATE sampleRate
#else
#define AUDIO_SAMPLE_RATE 16000U
#endif

// ----- System peripheral clock/reset registers -----
#define SYSTEM_PERIP_CLK_EN0_REG   0x600C0018
#define SYSTEM_PERIP_RST_EN0_REG   0x600C0020

// ----- RMT registers -----
#define RMT_CH0_CONF0_REG          0x60016020
#define RMT_SYS_CONF_REG           0x600160C0
#define RMT_INT_RAW_REG            0x60016070
#define RMT_INT_CLR_REG            0x6001607C
#define RMT_RAM_BASE               0x60016800

// ----- LEDC registers -----
#define LEDC_CH0_CONF0_REG         0x60019000
#define LEDC_CH0_DUTY_REG          0x60019008
#define LEDC_CH0_CONF1_REG         0x6001900C
#define LEDC_TIMER0_CONF_REG       0x600190A0
#define LEDC_INT_RAW_REG           0x600190C0
#define LEDC_INT_CLR_REG           0x600190CC
#define LEDC_CONF_REG              0x600190D0

// ============================================================================
// GPIO matrix / IO MUX registers
//
// The PDF explains that GPIO_FUNCn_OUT_SEL_CFG_REG chooses the internal
// peripheral signal routed to each pin, and IO_MUX_GPIOx_REG selects MCU_SEL.
// We define the exact registers we need for GPIO14 and GPIO33.
// ============================================================================

// GPIO matrix base for output selection registers on ESP32-S3
#define GPIO_FUNC0_OUT_SEL_CFG_REG 0x60004554

#define GPIO_FUNC14_OUT_SEL_CFG_REG (GPIO_FUNC0_OUT_SEL_CFG_REG + (14U * 4U))
#define GPIO_FUNC33_OUT_SEL_CFG_REG (GPIO_FUNC0_OUT_SEL_CFG_REG + (33U * 4U))

// IO_MUX base on ESP32-S3
#define REG_IO_MUX_BASE            0x60009000

#define IO_MUX_GPIO14_REG          (REG_IO_MUX_BASE + 0x3CU)
#define IO_MUX_GPIO33_REG          (REG_IO_MUX_BASE + 0x88U)

// ============================================================================
// Bit masks / field helpers
// ============================================================================

// ----- SYSTEM_PERIP_CLK_EN0_REG / SYSTEM_PERIP_RST_EN0_REG -----
#define SYSTEM_RMT_CLK_EN_BIT      (1U << 9)
#define SYSTEM_RMT_RST_BIT         (1U << 9)
#define SYSTEM_LEDC_CLK_EN_BIT     (1U << 11)
#define SYSTEM_LEDC_RST_BIT        (1U << 11)

// ----- RMT_SYS_CONF_REG -----
#define RMT_APB_FIFO_MASK_BIT      (1U << 0)
#define RMT_MEM_CLK_FORCE_ON_BIT   (1U << 1)
#define RMT_SCLK_DIV_NUM_MASK      (0xFFU << 24)
#define RMT_CLK_EN_BIT             (1UL << 31)

// ----- RMT_CH0_CONF0_REG -----
#define RMT_DIV_CNT_CH0_MASK       (0xFFU << 0)
#define RMT_TX_START_CH0_BIT       (1U << 9)
#define RMT_IDLE_OUT_LV_CH0_BIT    (1U << 12)
#define RMT_IDLE_OUT_EN_CH0_BIT    (1U << 13)
#define RMT_CARRIER_EN_CH0_BIT     (1U << 14)
#define RMT_CONF_UPDATE_CH0_BIT    (1U << 24)

// ----- RMT interrupts -----
#define RMT_CH0_TX_END_INT_RAW_BIT (1U << 0)
#define RMT_CH0_TX_END_INT_CLR_BIT (1U << 0)

// ----- LEDC_CONF_REG -----
#define LEDC_APB_CLK_SEL_BIT       (1U << 0)
#define LEDC_CLK_EN_BIT            (1UL << 31)

// ----- LEDC_TIMER0_CONF_REG -----
#define LEDC_DUTY_RES_S            0
#define LEDC_DUTY_RES_MASK         (0x1FU << LEDC_DUTY_RES_S)
#define LEDC_CLK_DIV_S             5
#define LEDC_CLK_DIV_MASK          (0x3FFFFU << LEDC_CLK_DIV_S)   // 18.8 fixed point
#define LEDC_TIMER0_RST_BIT        (1U << 23)
#define LEDC_TIMER0_PARA_UP_BIT    (1U << 25)

// ----- LEDC_CH0_CONF0_REG -----
#define LEDC_TIMER_SEL_CH0_MASK    (0x3U << 0)
#define LEDC_SIG_OUT_EN_CH0_BIT    (1U << 2)
#define LEDC_PARA_UP_CH0_BIT       (1U << 4)

// ----- LEDC_CH0_CONF1_REG -----
#define LEDC_DUTY_START_CH0_BIT    (1U << 31)

// ----- LEDC interrupts -----
#define LEDC_TIMER0_OVF_INT_RAW_BIT (1U << 10)
#define LEDC_TIMER0_OVF_INT_CLR_BIT (1U << 10)

// ----- GPIO_FUNCx_OUT_SEL_CFG_REG fields -----
#define GPIO_FUNC_OUT_SEL_MASK     0x1FFU
#define GPIO_FUNC_OEN_SEL_BIT      (1U << 9)
#define GPIO_FUNC_OEN_INV_SEL_BIT  (1U << 10)

// ----- IO_MUX fields -----
#define MCU_SEL_S                  12
#define MCU_SEL_MASK               (0x7U << MCU_SEL_S)

// ============================================================================
// GPIO function select values from the PDF
// ============================================================================

#define GPIO_FUNC_SEL_LEDC         73U
#define GPIO_FUNC_SEL_RMT          81U
#define IO_MUX_MCU_SEL_GPIO_MATRIX 2U

// ============================================================================
// WS2812 timing in RMT clock cycles
//
// PDF timing:
//   1-bit: high 800 ns, low 450 ns
//   0-bit: high 400 ns, low 850 ns
//
// We use APB 80 MHz and divider = 1, so one RMT tick = 12.5 ns.
//   800 ns / 12.5 ns = 64
//   450 ns / 12.5 ns = 36
//   400 ns / 12.5 ns = 32
//   850 ns / 12.5 ns = 68
// ============================================================================

#define WS2812_T1H_CYCLES          64U
#define WS2812_T1L_CYCLES          36U
#define WS2812_T0H_CYCLES          32U
#define WS2812_T0L_CYCLES          68U

// ============================================================================
// Global state
// ============================================================================

static uint32_t g_sample_period_us = 0U;
static bool g_audio_running = false;

// Color buffer for all LEDs.
// Requirement names the function as transmit_led_signal(uint32_t *colors),
// so we keep the whole LED string in an array of 24-bit colors.
static uint32_t g_led_colors[NUM_LEDS];

// ============================================================================
// Function prototypes
// ============================================================================

void setup_RMT();
void setup_LEDC();
void update_PWM(uint32_t initial, uint32_t sample);
void transmit_led_signal(uint32_t *colors);

static void configure_gpio_matrix_outputs();
static void write_rmt_item(uint32_t index, uint16_t dur0, uint16_t lvl0, uint16_t dur1, uint16_t lvl1);
static void encode_one_led_to_rmt_ram(uint32_t color);
static void wait_for_rmt_done_and_clear();
static void play_square_wave_once(float frequency_hz, uint32_t duration_ms);
static void play_audio_array_once();
static void stop_audio_output();
static void clear_led_buffer();
static void set_all_leds_to_color(uint32_t color);
static uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);
static uint32_t wheel_color(uint8_t pos);
static void amplitude_to_color(uint8_t value, uint8_t *r, uint8_t *g, uint8_t *b);
static uint32_t compute_visual_led_count(uint8_t sample);
static void update_visualization_from_sample(uint8_t sample);
static void run_setup_stage();

// ============================================================================
// Arduino setup / loop
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  g_sample_period_us = 1000000UL / AUDIO_SAMPLE_RATE;

  configure_gpio_matrix_outputs();
  setup_RMT();
  setup_LEDC();

  Serial.println("SETUP STAGE: register-level speaker + LED test starting");
  run_setup_stage();

#if HAS_AUDIO_ARRAY
  Serial.println("AUDIO ARRAY: playing array.h once with LED visualization");
  play_audio_array_once();
  stop_audio_output();
#else
  Serial.println("AUDIO ARRAY: array.h not found, skipping playback");
#endif

  Serial.println("DONE");
}

void loop() {
  delay(1000);
}

// ============================================================================
// GPIO routing
// ============================================================================

static void configure_gpio_matrix_outputs() {
  // --------------------------------------------------------------------------
  // Configure GPIO14 to output LEDC channel signal through the GPIO matrix.
  //
  // PDF page 6:
  //   GPIO14 -> LEDC -> GPIO_FUNC_OUT_SEL = 73
  //   OEN_INV_SEL = 0
  //   OEN_SEL = 0
  // PDF page 7:
  //   IO_MUX GPIO14 MCU_SEL = 2
  // --------------------------------------------------------------------------
  uint32_t reg14 = 0;
  reg14 |= (GPIO_FUNC_SEL_LEDC & GPIO_FUNC_OUT_SEL_MASK);
  // OEN_SEL = 0, OEN_INV_SEL = 0, so leave those bits cleared
  REG_WRITE(GPIO_FUNC14_OUT_SEL_CFG_REG, reg14);

  uint32_t iomux14 = REG_READ(IO_MUX_GPIO14_REG);
  iomux14 &= ~MCU_SEL_MASK;
  iomux14 |= (IO_MUX_MCU_SEL_GPIO_MATRIX << MCU_SEL_S);
  REG_WRITE(IO_MUX_GPIO14_REG, iomux14);

  // --------------------------------------------------------------------------
  // Configure GPIO33 to output RMT channel 0 signal through the GPIO matrix.
  //
  // PDF page 6:
  //   GPIO33 -> RMT -> GPIO_FUNC_OUT_SEL = 81
  // --------------------------------------------------------------------------
  uint32_t reg33 = 0;
  reg33 |= (GPIO_FUNC_SEL_RMT & GPIO_FUNC_OUT_SEL_MASK);
  REG_WRITE(GPIO_FUNC33_OUT_SEL_CFG_REG, reg33);

  uint32_t iomux33 = REG_READ(IO_MUX_GPIO33_REG);
  iomux33 &= ~MCU_SEL_MASK;
  iomux33 |= (IO_MUX_MCU_SEL_GPIO_MATRIX << MCU_SEL_S);
  REG_WRITE(IO_MUX_GPIO33_REG, iomux33);
}

// ============================================================================
// RMT setup
// ============================================================================

void setup_RMT() {
  // Turn on RMT peripheral clock
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_RMT_CLK_EN_BIT);

  // Put RMT through a clean reset pulse, then release reset
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST_BIT);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST_BIT);

  // Configure RMT_SYS_CONF_REG according to PDF
  uint32_t sys_conf = REG_READ(RMT_SYS_CONF_REG);

  // APB FIFO mask = 0
  sys_conf &= ~RMT_APB_FIFO_MASK_BIT;

  // Clock enable = 1
  sys_conf |= RMT_CLK_EN_BIT;

  // Force memory clock on = 1
  sys_conf |= RMT_MEM_CLK_FORCE_ON_BIT;

  // SCLK divider = 0 => divide by (0 + 1) = 1
  sys_conf &= ~RMT_SCLK_DIV_NUM_MASK;

  REG_WRITE(RMT_SYS_CONF_REG, sys_conf);

  // Configure channel 0
  uint32_t ch0 = REG_READ(RMT_CH0_CONF0_REG);

  // Disable carrier
  ch0 &= ~RMT_CARRIER_EN_CH0_BIT;

  // Divider = 1
  ch0 &= ~RMT_DIV_CNT_CH0_MASK;
  ch0 |= 1U;

  // Idle output enable = 1
  ch0 |= RMT_IDLE_OUT_EN_CH0_BIT;

  // Idle level = 0
  ch0 &= ~RMT_IDLE_OUT_LV_CH0_BIT;

  REG_WRITE(RMT_CH0_CONF0_REG, ch0);

  // Commit settings
  REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_CONF_UPDATE_CH0_BIT);

  // Clear old TX-done interrupt status
  REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_CLR_BIT);
}

// ============================================================================
// LEDC setup
// ============================================================================

void setup_LEDC() {
  // Turn on LEDC peripheral clock
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_LEDC_CLK_EN_BIT);

  // Reset LEDC, then take it out of reset
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);

  // LEDC_CONF_REG:
  //   bit 31 = 1 => enable LEDC register clock
  //   bit 0  = 1 => select APB clock
  uint32_t ledc_conf = 0;
  ledc_conf |= LEDC_CLK_EN_BIT;
  ledc_conf |= LEDC_APB_CLK_SEL_BIT;
  REG_WRITE(LEDC_CONF_REG, ledc_conf);

  // Compute 18.8 fixed-point divider:
  // divider = 80e6 / sampleRate
  // Store as (integer_part << 8) | fractional_part
  double divider_real = 80000000.0 / (double)AUDIO_SAMPLE_RATE;
  uint32_t divider_int = (uint32_t)floor(divider_real);
  uint32_t divider_frac = (uint32_t)((divider_real - (double)divider_int) * 256.0 + 0.5);

  if (divider_frac > 255U) {
    divider_frac = 0U;
    divider_int += 1U;
  }

  uint32_t divider_fixed = (divider_int << 8) | divider_frac;

  uint32_t timer0 = 0;

  // Duty resolution = 8 bits
  timer0 |= ((PWM_RES_BITS & 0x1FU) << LEDC_DUTY_RES_S);

  // Timer reset bit = 0
  timer0 &= ~LEDC_TIMER0_RST_BIT;

  // Divider field
  timer0 |= ((divider_fixed & 0x3FFFFU) << LEDC_CLK_DIV_S);

  REG_WRITE(LEDC_TIMER0_CONF_REG, timer0);

  // Commit timer settings
  REG_SET_BIT(LEDC_TIMER0_CONF_REG, LEDC_TIMER0_PARA_UP_BIT);

  // Channel 0:
  //   enable output
  //   select timer 0
  uint32_t ch0_conf0 = 0;
  ch0_conf0 &= ~LEDC_TIMER_SEL_CH0_MASK;   // timer 0
  ch0_conf0 |= LEDC_SIG_OUT_EN_CH0_BIT;    // enable LEDC output
  REG_WRITE(LEDC_CH0_CONF0_REG, ch0_conf0);

  // Commit channel parameters
  REG_SET_BIT(LEDC_CH0_CONF0_REG, LEDC_PARA_UP_CH0_BIT);

  // Clear old timer overflow interrupt
  REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_CLR_BIT);

  // Start with duty = 0 on first update
  update_PWM(1U, 0U);
}

// ============================================================================
// Required PWM update function
//
// The PDF requires:
//   void update_PWM(uint32_t initial, uint32_t sample)
//
// Behavior:
// - if initial != 0, update immediately
// - otherwise check LEDC_INT_RAW_REG to see if a PWM period completed
// - if not ready, return without writing any LEDC peripheral registers
// ============================================================================

void update_PWM(uint32_t initial, uint32_t sample) {
  if (!initial) {
    uint32_t raw = REG_READ(LEDC_INT_RAW_REG);
    if ((raw & LEDC_TIMER0_OVF_INT_RAW_BIT) == 0U) {
      // Not time to update yet, so return immediately exactly as required.
      return;
    }

    // Clear timer overflow interrupt since we are about to consume this period.
    REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_CLR_BIT);
  } else {
    // For the first sample, clear any stale interrupt state.
    REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_CLR_BIT);
  }

  // Write duty cycle into whole bits of LEDC_CH0_DUTY_REG.
  // PDF says shift whole bits left by 4 to zero the fractional bits.
  REG_WRITE(LEDC_CH0_DUTY_REG, ((sample & 0xFFU) << 4));

  // Commit the duty register contents
  REG_SET_BIT(LEDC_CH0_CONF0_REG, LEDC_PARA_UP_CH0_BIT);

  // Start duty update
  REG_SET_BIT(LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0_BIT);

  g_audio_running = true;
}

// ============================================================================
// RMT RAM helper
//
// One 32-bit RMT RAM word:
//   [14:0]  duration0
//   [15]    level0
//   [30:16] duration1
//   [31]    level1
// ============================================================================

static void write_rmt_item(uint32_t index, uint16_t dur0, uint16_t lvl0, uint16_t dur1, uint16_t lvl1) {
  volatile uint32_t *rmt_ram = (volatile uint32_t *)RMT_RAM_BASE;

  uint32_t word = 0;
  word |= ((uint32_t)(dur0 & 0x7FFFU) << 0);
  word |= ((uint32_t)(lvl0 & 0x1U)    << 15);
  word |= ((uint32_t)(dur1 & 0x7FFFU) << 16);
  word |= ((uint32_t)(lvl1 & 0x1U)    << 31);

  rmt_ram[index] = word;
}

// ============================================================================
// Convert one 24-bit color to 24 RMT entries in GRB order
// ============================================================================

static void encode_one_led_to_rmt_ram(uint32_t color) {
  uint8_t g = (uint8_t)((color >> 8) & 0xFFU);
  uint8_t r = (uint8_t)((color >> 16) & 0xFFU);
  uint8_t b = (uint8_t)(color & 0xFFU);

  uint8_t bytes[3] = { g, r, b };

  uint32_t idx = 0U;

  for (uint32_t byte_i = 0U; byte_i < 3U; ++byte_i) {
    for (int bit = 7; bit >= 0; --bit) {
      bool one = (((bytes[byte_i] >> bit) & 0x01U) != 0U);

      if (one) {
        // Logic-1: high 800 ns, low 450 ns
        write_rmt_item(idx, WS2812_T1H_CYCLES, 1U, WS2812_T1L_CYCLES, 0U);
      } else {
        // Logic-0: high 400 ns, low 850 ns
        write_rmt_item(idx, WS2812_T0H_CYCLES, 1U, WS2812_T0L_CYCLES, 0U);
      }

      ++idx;
    }
  }

  // 25th word is zero termination, per PDF
  write_rmt_item(24U, 0U, 0U, 0U, 0U);
}

static void wait_for_rmt_done_and_clear() {
  while ((REG_READ(RMT_INT_RAW_REG) & RMT_CH0_TX_END_INT_RAW_BIT) == 0U) {
    // Wait until channel 0 finishes transmitting.
  }

  REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_CLR_BIT);
}

// ============================================================================
// Required LED transmit function
//
// The PDF names the function:
//   void transmit_led_signal(uint32_t *colors)
//
// The PDF text also says to write 24 entries + zero termination, transmit,
// wait for completion, then repeat. This implementation does exactly that
// for all 100 LEDs on the board.
// ============================================================================

void transmit_led_signal(uint32_t *colors) {
  for (uint32_t led = 0U; led < NUM_LEDS; ++led) {
    encode_one_led_to_rmt_ram(colors[led]);

    // Commit RMT RAM contents
    REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_CONF_UPDATE_CH0_BIT);

    // Start transmission
    REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_TX_START_CH0_BIT);

    // Wait until the 24 bits for this LED have finished
    wait_for_rmt_done_and_clear();
  }

  // Reset/latch time for WS2812
  delayMicroseconds(WS2812_RESET_US);
}

// ============================================================================
// Audio helpers
// ============================================================================

static void stop_audio_output() {
  update_PWM(1U, 0U);
  g_audio_running = false;
}

static void play_square_wave_once(float frequency_hz, uint32_t duration_ms) {
  uint32_t total_samples = (duration_ms * AUDIO_SAMPLE_RATE) / 1000U;
  if (total_samples == 0U) {
    total_samples = 1U;
  }

  float phase = 0.0f;
  float phase_step = frequency_hz / (float)AUDIO_SAMPLE_RATE;

  bool first = true;

  for (uint32_t i = 0U; i < total_samples; ++i) {
    uint8_t sample = (phase < 0.5f) ? 255U : 0U;

    if (first) {
      update_PWM(1U, sample);
      first = false;
    } else {
      // Keep calling until the hardware period boundary says it is time.
      while (true) {
        uint32_t before = REG_READ(LEDC_INT_RAW_REG);
        update_PWM(0U, sample);
        uint32_t after = REG_READ(LEDC_INT_RAW_REG);

        // If overflow was present, update_PWM consumed it and wrote the sample.
        // If not, we spin until it becomes ready.
        if ((before & LEDC_TIMER0_OVF_INT_RAW_BIT) || !(after & LEDC_TIMER0_OVF_INT_RAW_BIT)) {
          break;
        }
      }
    }
  }
}

static void play_audio_array_once() {
#if HAS_AUDIO_ARRAY
  uint32_t sample_count = sizeof(sampleArray) / sizeof(sampleArray[0]);
  bool first = true;

  for (uint32_t i = 0U; i < sample_count; ++i) {
    uint8_t sample = (uint8_t)(sampleArray[i] & 0xFFU);

    if (first) {
      update_PWM(1U, sample);
      first = false;
    } else {
      while ((REG_READ(LEDC_INT_RAW_REG) & LEDC_TIMER0_OVF_INT_RAW_BIT) == 0U) {
        // wait for one PWM period to complete
      }
      update_PWM(0U, sample);
    }

    // Update the LED visualization from this sample
    update_visualization_from_sample(sample);
  }
#endif
}

// ============================================================================
// LED helpers / visualization
// ============================================================================

static void clear_led_buffer() {
  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = 0U;
  }
}

static void set_all_leds_to_color(uint32_t color) {
  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = color;
  }
}

static uint32_t make_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static uint32_t wheel_color(uint8_t pos) {
  pos = 255U - pos;

  if (pos < 85U) {
    return make_color((uint8_t)(255U - pos * 3U), 0U, (uint8_t)(pos * 3U));
  }

  if (pos < 170U) {
    pos -= 85U;
    return make_color(0U, (uint8_t)(pos * 3U), (uint8_t)(255U - pos * 3U));
  }

  pos -= 170U;
  return make_color((uint8_t)(pos * 3U), (uint8_t)(255U - pos * 3U), 0U);
}

// This follows the color-map idea shown in the PDF.
static void amplitude_to_color(uint8_t value, uint8_t *r, uint8_t *g, uint8_t *b) {
  if (value <= 51U) {
    *r = 0U;
    *g = (uint8_t)(value * 5U);
    *b = 255U;
  } else if (value <= 102U) {
    *r = 0U;
    *g = 255U;
    *b = (uint8_t)(255U - (value - 51U) * 5U);
  } else if (value <= 153U) {
    *r = (uint8_t)((value - 102U) * 5U);
    *g = 255U;
    *b = 0U;
  } else if (value <= 204U) {
    *r = 255U;
    *g = (uint8_t)(255U - (value - 153U) * 5U);
    *b = 0U;
  } else {
    *r = 255U;
    *g = 0U;
    *b = (value < 230U) ? 0U : (uint8_t)((value - 204U) * 4U);
  }
}

// This follows the PDF's log-scale visualization idea.
static uint32_t compute_visual_led_count(uint8_t sample) {
  double log_val = log10((double)sample + 1.0) / log10(256.0);
  long n = lround(log_val * 100.0);

  if (n < 0L) {
    n = 0L;
  }
  if (n > 100L) {
    n = 100L;
  }

  return (uint32_t)n;
}

static void update_visualization_from_sample(uint8_t sample) {
  uint8_t r, g, b;
  amplitude_to_color(sample, &r, &g, &b);

  uint32_t lit = compute_visual_led_count(sample);
  uint32_t color = make_color(r, g, b);

  for (uint32_t i = 0U; i < NUM_LEDS; ++i) {
    g_led_colors[i] = (i < lit) ? color : 0U;
  }

  transmit_led_signal(g_led_colors);
}

// ============================================================================
// Demo / setup stage
// ============================================================================

static void run_setup_stage() {
  // Clear LEDs first
  clear_led_buffer();
  transmit_led_signal(g_led_colors);

  // Short speaker test
  play_square_wave_once(SPEAKER_TEST_HZ, SPEAKER_TEST_MS);
  stop_audio_output();

  // LED rainbow sweep, one LED at a time
  for (uint32_t led_index = 0U; led_index < NUM_LEDS; ++led_index) {
    for (uint32_t step = 0U; step < COLOR_STEPS; ++step) {
      clear_led_buffer();

      uint8_t wheel_pos = (uint8_t)((step * 256U) / COLOR_STEPS);
      g_led_colors[led_index] = wheel_color(wheel_pos);

      transmit_led_signal(g_led_colors);
      delay(COLOR_STEP_DELAY_MS);
    }
  }

  clear_led_buffer();
  transmit_led_signal(g_led_colors);
}