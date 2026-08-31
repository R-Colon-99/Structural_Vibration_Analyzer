// V4 - Auto-detected MPU6050 binary vibration logger
// Structural Vibration Analyzer

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// ESP32 + PCA9548A + AUTO-DETECTED MPU6050 BINARY LOGGER
// ============================================================
//
// Hardware-safe baseline configuration:
//   I2C clock:        400 kHz
//   Output rate:      500 Hz
//   MPU6050 DLPF:     CFG=1 (~184 Hz accel, ~188 Hz gyro)
//   Accelerometer:    +/-8 g
//   Gyroscope:        +/-250 deg/s
//   Serial:           921600 baud
//
// The PCA9548A channels 0, 1, and 2 are scanned at startup.
// Only detected MPU6050 sensors are configured and streamed.
//
// IMPORTANT:
// - Accelerometer samples are NOT gravity-zeroed in firmware.
//   Static/DC removal is performed later in the PC analysis.
// - Gyroscope zero-rate bias is estimated while stationary and removed.
// - A binary configuration packet identifies the active PCA channels.
// - Data packet payload size varies with the number of detected sensors.
//
// Binary frame:
//   sync0, sync1, packet_type, payload_size, payload..., checksum
//
// CONFIG payload:
//   sensor_count [u8]
//   active PCA channels [u8 x sensor_count]
//
// DATA payload:
//   t_us [u32]
//   sample_index [u32]
//   for each active sensor, in CONFIG order:
//       ax ay az gx gy gz [int16 each]
//
// All multibyte numeric fields are little-endian.
// ============================================================

#define MPU_ADDR 0x68
#define PCA_ADDR 0x70

#define SDA_PIN 21
#define SCL_PIN 22

const uint32_t SERIAL_BAUD = 921600;

const uint32_t I2C_CLOCK_HZ = 400000;

const float SAMPLE_RATE_HZ = 500.0f;
const uint32_t SAMPLE_PERIOD_US =
    (uint32_t)(1000000.0f / SAMPLE_RATE_HZ);

// Maximum number of sensors/channels used by this project.
const int MAX_SENSORS = 3;
const uint8_t POSSIBLE_SENSOR_CHANNELS[MAX_SENSORS] = {0, 1, 2};

// Gyro zero-rate calibration samples.
// At 500 Hz this is approximately 2 seconds per sensor.
const int CALIBRATION_SAMPLES = 1000;

// MPU6050 register configuration.
const uint8_t DLPF_CFG = 0x01;       // ~184 Hz accel BW, ~188 Hz gyro BW
const uint8_t GYRO_CONFIG = 0x00;    // +/-250 deg/s
const uint8_t ACCEL_CONFIG = 0x10;   // +/-8 g
const uint8_t SMPLRT_DIV = 0x01;     // 1 kHz / (1 + 1) = 500 Hz

const float ACCEL_SCALE = 4096.0f;   // +/-8 g raw counts per g
const float GYRO_SCALE = 131.0f;     // +/-250 deg/s raw counts per deg/s

const uint8_t SYNC0 = 0xAA;
const uint8_t SYNC1 = 0x55;

const uint8_t PACKET_TYPE_DATA = 0x01;
const uint8_t PACKET_TYPE_CONFIG = 0x02;

const uint8_t BYTES_PER_SENSOR = 6 * 2;
const uint8_t BASE_DATA_PAYLOAD_SIZE = 4 + 4;  // t_us + sample_index

struct GyroOffset {
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

uint32_t streamStartUs = 0;
uint32_t sampleIndex = 0;
uint32_t missedDeadlines = 0;

GyroOffset gyroOffsets[MAX_SENSORS];
uint8_t activeChannels[MAX_SENSORS];
uint8_t activeSensorCount = 0;
uint8_t dataPayloadSize = BASE_DATA_PAYLOAD_SIZE;

// ============================================================
// I2C / MPU HELPERS
// ============================================================

void selectPCAChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1U << channel);
  Wire.endTransmission(true);
}

void disablePCAChannels() {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

bool writeMPU(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readMPU(
    int16_t &ax, int16_t &ay, int16_t &az,
    int16_t &gx, int16_t &gy, int16_t &gz) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const int bytesRead = Wire.requestFrom((int)MPU_ADDR, 14, (int)true);

  if (bytesRead != 14) {
    return false;
  }

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());

  Wire.read();
  Wire.read();  // temperature ignored

  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());

  return true;
}

bool mpuPresentOnSelectedChannel() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission(true) == 0;
}

bool setupMPUOnSelectedChannel() {
  bool ok = true;

  ok &= writeMPU(0x6B, 0x00);          // wake up
  delay(100);

  ok &= writeMPU(0x1A, DLPF_CFG);
  ok &= writeMPU(0x1B, GYRO_CONFIG);
  ok &= writeMPU(0x1C, ACCEL_CONFIG);
  ok &= writeMPU(0x19, SMPLRT_DIV);

  return ok;
}

// ============================================================
// PACKET HELPERS
// ============================================================

int16_t clampToInt16(float value) {
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)lround(value);
}

void appendU8(uint8_t *buffer, uint8_t &index, uint8_t value) {
  buffer[index++] = value;
}

void appendU32(uint8_t *buffer, uint8_t &index, uint32_t value) {
  buffer[index++] = (uint8_t)(value & 0xFF);
  buffer[index++] = (uint8_t)((value >> 8) & 0xFF);
  buffer[index++] = (uint8_t)((value >> 16) & 0xFF);
  buffer[index++] = (uint8_t)((value >> 24) & 0xFF);
}

void appendI16(uint8_t *buffer, uint8_t &index, int16_t value) {
  buffer[index++] = (uint8_t)(value & 0xFF);
  buffer[index++] = (uint8_t)((value >> 8) & 0xFF);
}

uint8_t calculateChecksum(const uint8_t *buffer, uint8_t length) {
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < length; i++) {
    checksum ^= buffer[i];
  }

  return checksum;
}

void sendFrame(uint8_t packetType, const uint8_t *payload, uint8_t payloadSize) {
  const uint8_t frameSize = 2 + 1 + 1 + payloadSize + 1;
  uint8_t frame[2 + 1 + 1 + 64 + 1];
  uint8_t idx = 0;

  appendU8(frame, idx, SYNC0);
  appendU8(frame, idx, SYNC1);
  appendU8(frame, idx, packetType);
  appendU8(frame, idx, payloadSize);

  for (uint8_t i = 0; i < payloadSize; i++) {
    appendU8(frame, idx, payload[i]);
  }

  frame[idx++] = calculateChecksum(frame, idx);
  Serial.write(frame, frameSize);
}

// ============================================================
// SENSOR DETECTION / CALIBRATION
// ============================================================

void detectSensors() {
  activeSensorCount = 0;

  Serial.println("# Scanning PCA9548A channels for MPU6050 sensors...");

  for (int i = 0; i < MAX_SENSORS; i++) {
    const uint8_t channel = POSSIBLE_SENSOR_CHANNELS[i];

    selectPCAChannel(channel);
    delay(5);

    Serial.print("# Checking PCA channel ");
    Serial.print(channel);
    Serial.print(": ");

    if (mpuPresentOnSelectedChannel()) {
      if (setupMPUOnSelectedChannel()) {
        Serial.println("MPU6050 found and configured");
        activeChannels[activeSensorCount++] = channel;
      } else {
        Serial.println("MPU6050 found but configuration failed");
      }
    } else {
      Serial.println("NOT FOUND");
    }
  }

  dataPayloadSize =
      BASE_DATA_PAYLOAD_SIZE + (activeSensorCount * BYTES_PER_SENSOR);

  Serial.print("# Detected sensor count: ");
  Serial.println(activeSensorCount);

  Serial.print("# Active PCA channels: ");
  if (activeSensorCount == 0) {
    Serial.println("none");
  } else {
    for (int i = 0; i < activeSensorCount; i++) {
      Serial.print(activeChannels[i]);
      if (i < activeSensorCount - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
  }

  Serial.print("# Binary data payload size: ");
  Serial.println(dataPayloadSize);
}

void calibrateGyros() {
  Serial.print("# Calibrating gyro zero-rate bias for ");
  Serial.print(activeSensorCount);
  Serial.println(" MPU6050 sensor(s). Keep all sensors still.");

  for (int sensor = 0; sensor < activeSensorCount; sensor++) {
    selectPCAChannel(activeChannels[sensor]);

    int64_t gxSum = 0;
    int64_t gySum = 0;
    int64_t gzSum = 0;
    int validSamples = 0;

    int16_t ax, ay, az, gx, gy, gz;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      if (readMPU(ax, ay, az, gx, gy, gz)) {
        gxSum += gx;
        gySum += gy;
        gzSum += gz;
        validSamples++;
      }

      delayMicroseconds(SAMPLE_PERIOD_US);
    }

    if (validSamples == 0) {
      Serial.print("# WARNING: No valid gyro calibration samples for PCA channel ");
      Serial.println(activeChannels[sensor]);
      continue;
    }

    gyroOffsets[sensor].gx = gxSum / (float)validSamples;
    gyroOffsets[sensor].gy = gySum / (float)validSamples;
    gyroOffsets[sensor].gz = gzSum / (float)validSamples;

    Serial.print("# PCA channel ");
    Serial.print(activeChannels[sensor]);
    Serial.println(" gyro calibration complete.");
  }

  Serial.println("# Gyro calibration complete.");
}

// ============================================================
// CONFIG + DATA STREAM
// ============================================================

void sendConfigurationFrame() {
  uint8_t payload[1 + MAX_SENSORS];
  uint8_t idx = 0;

  appendU8(payload, idx, activeSensorCount);

  for (int i = 0; i < activeSensorCount; i++) {
    appendU8(payload, idx, activeChannels[i]);
  }

  sendFrame(PACKET_TYPE_CONFIG, payload, idx);
}

void sendDataFrame() {
  uint8_t payload[
      BASE_DATA_PAYLOAD_SIZE + (MAX_SENSORS * BYTES_PER_SENSOR)];
  uint8_t idx = 0;

  appendU32(payload, idx, micros() - streamStartUs);
  appendU32(payload, idx, sampleIndex++);

  for (int sensor = 0; sensor < activeSensorCount; sensor++) {
    selectPCAChannel(activeChannels[sensor]);

    int16_t rawAx = 0;
    int16_t rawAy = 0;
    int16_t rawAz = 0;
    int16_t rawGx = 0;
    int16_t rawGy = 0;
    int16_t rawGz = 0;

    const bool ok =
        readMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    // Acceleration is intentionally kept close to the sensor measurement.
    // Gravity/DC removal is performed in the PC analysis.
    const int16_t ax = ok ? rawAx : 0;
    const int16_t ay = ok ? rawAy : 0;
    const int16_t az = ok ? rawAz : 0;

    // Remove only stationary gyro zero-rate bias.
    const int16_t gx =
        ok ? clampToInt16(rawGx - gyroOffsets[sensor].gx) : 0;
    const int16_t gy =
        ok ? clampToInt16(rawGy - gyroOffsets[sensor].gy) : 0;
    const int16_t gz =
        ok ? clampToInt16(rawGz - gyroOffsets[sensor].gz) : 0;

    appendI16(payload, idx, ax);
    appendI16(payload, idx, ay);
    appendI16(payload, idx, az);
    appendI16(payload, idx, gx);
    appendI16(payload, idx, gy);
    appendI16(payload, idx, gz);
  }

  sendFrame(PACKET_TYPE_DATA, payload, idx);
}

// ============================================================
// ARDUINO SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println("# ESP32 AUTO-DETECT MPU6050 PCA9548A binary logger - V4");
  Serial.print("# Baud: ");
  Serial.println(SERIAL_BAUD);
  Serial.print("# I2C clock: ");
  Serial.println(I2C_CLOCK_HZ);
  Serial.print("# Target sample rate: ");
  Serial.println(SAMPLE_RATE_HZ, 2);
  Serial.print("# MPU6050 DLPF_CFG: ");
  Serial.println(DLPF_CFG);
  Serial.println("# Accel bandwidth: approximately 184 Hz");
  Serial.println("# Gyro bandwidth: approximately 188 Hz");
  Serial.println("# Accelerometer range: +/-8 g");
  Serial.println("# Gyroscope range: +/-250 deg/s");

  detectSensors();

  if (activeSensorCount == 0) {
    Serial.println(
        "# ERROR: No MPU6050 sensors detected. "
        "Check wiring, power, and PCA9548A channels.");
    Serial.println(
        "# Firmware halted. Reset the ESP32 after fixing the wiring.");

    disablePCAChannels();

    while (true) {
      delay(1000);
    }
  }

  calibrateGyros();

  sampleIndex = 0;
  missedDeadlines = 0;
  streamStartUs = micros();

  Serial.println("# Binary stream starting now.");
  Serial.println("START_BINARY");
  Serial.flush();

  // Send the active channel map before normal data packets.
  sendConfigurationFrame();
}

void loop() {
  static uint32_t nextSampleUs = micros();

  const uint32_t now = micros();

  if ((int32_t)(now - nextSampleUs) >= 0) {
    // If execution falls more than one whole period behind, move the
    // schedule forward instead of producing a burst of catch-up samples.
    const uint32_t lateness = now - nextSampleUs;

    if (lateness >= SAMPLE_PERIOD_US) {
      const uint32_t skippedPeriods = lateness / SAMPLE_PERIOD_US;
      missedDeadlines += skippedPeriods;
      nextSampleUs += skippedPeriods * SAMPLE_PERIOD_US;
    }

    sendDataFrame();
    nextSampleUs += SAMPLE_PERIOD_US;
  }
}
