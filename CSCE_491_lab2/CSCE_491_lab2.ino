#include <Arduino.h>
#include <math.h>

#define SCL 1
#define SDA 2

// 7 bit I2C addr for MPU6050
static const uint8_t MPU_ADDR = 0x68;

// MPU6050 registers
static const uint8_t REG_PWR_MGMT_1 = 0x6B; // power management
static const uint8_t REG_ACCEL_XOUT_H = 0x3B; // accel x high

// open drain lets you pull it low and goes high when released
static void sda_output() {
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
}

// input with pull up
static void sda_input() {
  digitalWrite(SDA, HIGH); // release line before switching to input
  pinMode(SDA, INPUT_PULLUP); // read SDA while keeping it up
}

// while SCL is high SDA is pulled low then is clocked by SCL low
static void i2c_start() {
  sda_output();
  digitalWrite(SDA, HIGH);
  digitalWrite(SCL, HIGH);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
}

// while SCL is high pull SDA from low to high
static void i2c_stop() {
  sda_output();
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, HIGH);
  digitalWrite(SDA, HIGH);
}

static bool i2c_write_byte(uint8_t data) {
  sda_output();
  // reads bits 7-0 if current bit is 1 release, if 0 pull low
  for (int i = 7; i >= 0; --i) {
    if (data & (1 << i)) {
      digitalWrite(SDA, HIGH);
    } else {
      digitalWrite(SDA, LOW);
    }
    digitalWrite(SCL, HIGH);
    digitalWrite(SCL, LOW);
  }

  // ACK bit - release SDA slave drives SDA low
  sda_input();
  digitalWrite(SCL, HIGH);
  bool ack = (digitalRead(SDA) == LOW);
  digitalWrite(SCL, LOW);
  sda_output();
  return ack;
}

// if send_ack is true master will pull SDA low and request more
// if send_ack is false masters leaves SDA high and end.
static uint8_t i2c_read_byte(bool send_ack) {
  uint8_t data = 0;
  sda_input();
  // read MSB
  for (int i = 7; i >= 0; --i) {
    digitalWrite(SCL, HIGH);
    if (digitalRead(SDA)) { // if SDA high bit is 1
      data |= (1 << i);
    }
    digitalWrite(SCL, LOW); // move to next
  }

  // ACK/NACK bit
  sda_output();
  // if ack is true pull SDA low otherwise leave high
  if (send_ack) {
    digitalWrite(SDA, LOW);
  } else {
    digitalWrite(SDA, HIGH);
  }
  digitalWrite(SCL, HIGH);
  digitalWrite(SCL, LOW);
  return data;
}


// returns true only if all bytes got ack
static bool i2c_write_register(uint8_t dev_addr, uint8_t reg, uint8_t value) {
  Serial.printf("I2C WRITE dev=0x%02X reg=0x%02X val=0x%02X\n", dev_addr, reg, value);
  i2c_start();
  // device addr with write
  bool ack1 = i2c_write_byte((dev_addr << 1) | 0);
  if (!ack1) {
    Serial.println("NACK after device address (write)");
  }
  // register addr
  bool ack2 = i2c_write_byte(reg);
  if (!ack2) {
    Serial.println("NACK after register address");
  }
  // reg data byte
  bool ack3 = i2c_write_byte(value);
  if (!ack3) {
    Serial.println("NACK after data byte");
  }
  i2c_stop();
  return ack1 && ack2 && ack3;
}

static bool i2c_read_registers(uint8_t dev_addr, uint8_t start_reg, uint8_t *buf, size_t len) {
  if (len == 0) return true;
  Serial.printf("I2C READ dev=0x%02X start_reg=0x%02X len=%u\n", dev_addr, start_reg, (unsigned)len);

  // Transaction 1: write start register
  i2c_start();
  // device addr with write
  bool ack1 = i2c_write_byte((dev_addr << 1) | 0);
  if (!ack1) {
    Serial.println("NACK after device address (write)");
  }
  // reg
  bool ack2 = i2c_write_byte(start_reg);
  if (!ack2) {
    Serial.println("NACK after register address");
  }
  i2c_stop();

  // Transaction 2: read data
  i2c_start();
  bool ack3 = i2c_write_byte((dev_addr << 1) | 1);
  if (!ack3) {
    Serial.println("NACK after device address (read)");
  }
  // read until second to last byte
  for (size_t i = 0; i < len; ++i) {
    bool send_ack = (i + 1 < len);
    buf[i] = i2c_read_byte(send_ack);
  }

  i2c_stop();
  return ack1 && ack2 && ack3;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting MPU6050 initialization...");

// release both high
  pinMode(SCL, OUTPUT_OPEN_DRAIN);
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL, HIGH);
  digitalWrite(SDA, HIGH);

  // write 0 to power management register 0x6B on startup.
  delay(50);
  bool init_ok = i2c_write_register(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
  if (init_ok) {
    Serial.println("MPU6050 woke up successfully");
  } else {
    Serial.println("MPU6050 initialization FAILED - check wiring");
  }

  delay(100);
  uint8_t who_am_i = 0;
  if (i2c_read_registers(MPU_ADDR, 0x75, &who_am_i, 1)) {
    Serial.printf("WHO_AM_I register: 0x%02X (expected 0x68)\n", who_am_i);
  } else {
    Serial.println("Failed to read WHO_AM_I register");
  }
  Serial.println("Starting main loop...");
}

void loop() {
  uint8_t data[6] = {0};
  // read 6 bytes starting at 0x3B
  if (i2c_read_registers(MPU_ADDR, REG_ACCEL_XOUT_H, data, sizeof(data))) {
    // combine into 16 bit
    int16_t ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t az = (int16_t)((data[4] << 8) | data[5]);

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;
    
    // convert to degrees
    float angle_xy = atan2(ax_g, ay_g) * RAD_TO_DEG;
    float angle_xz = atan2(ax_g, az_g) * RAD_TO_DEG;
    float angle_yz = atan2(ay_g, az_g) * RAD_TO_DEG;

    Serial.printf(
        "RAW ax=%d ay=%d az=%d | g ax=%.4f ay=%.4f az=%.4f | ANG XY=%.2f XZ=%.2f YZ=%.2f\n",
        ax, ay, az, ax_g, ay_g, az_g, angle_xy, angle_xz, angle_yz);
  } else {
    Serial.println("I2C read failed");
  }

  delay(1000);
}
