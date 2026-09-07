// V7 - Dual-mode (FAST_ACCEL / FULL_DIAGNOSTIC) auto-detected MPU6050
// binary vibration logger with I2C health monitoring.
// Structural Vibration Analyzer

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// ESP32 + PCA9548A + AUTO-DETECTED MPU6050 BINARY LOGGER
// ============================================================
//
// Validated acquisition configuration (see Data/validation run history):
//   I2C clock:                 400 kHz
//   Nominal ESP32 poll rate:   see SAMPLE_RATE_HZ below (currently 800 Hz)
//   MPU6050 DLPF_CFG:          1 (~184 Hz accel BW, ~188 Hz gyro BW)
//   MPU6050 SMPLRT_DIV:        0
//   MPU6050 internal register-update rate: computed below from DLPF_CFG/
//                              SMPLRT_DIV, not hardcoded (see
//                              MPU_INTERNAL_RATE_HZ).
//   Accelerometer:             +/-8 g
//   Gyroscope:                 +/-250 deg/s (FULL_DIAGNOSTIC mode only)
//   Serial:                    921600 baud
//
// This firmware supports two acquisition modes, selected by the
// ACQUISITION_MODE constant below (change and reflash to switch - a
// runtime-switchable mode would require adding a serial command
// protocol, which is not justified for how infrequently this changes):
//
//   ACQ_MODE_FAST_ACCEL (default) - accelerometer-only 6-byte reads per
//     sensor. This is the normal mode for vibration/FFT analysis. Gyro
//     calibration is skipped entirely in this mode (no gyro data is
//     acquired, so there is nothing to calibrate), which also removes
//     the ~2 second calibration delay at startup.
//
//   ACQ_MODE_FULL_DIAGNOSTIC - full 14-byte accel+temperature+gyro
//     reads per sensor (temperature is read but not transmitted, as
//     before). Gyro zero-rate bias is calibrated at startup as before.
//     Use this mode when gyro data is specifically needed (e.g.
//     investigating torsional modes or checking mounting integrity) -
//     analyze_fft.py's FFT/FRF pipeline does not currently use gyro
//     data, so FAST_ACCEL is the default for routine work.
//
// The PCA9548A channels 0, 1, and 2 are scanned at startup. Only
// detected MPU6050 sensors are configured and streamed - the firmware
// does not assume any particular sensor count; it works with however
// many are actually present (currently 2 in the fielded hardware, but
// nothing here is hardcoded to that number).
//
// IMPORTANT:
// - Accelerometer samples are NOT gravity-zeroed in firmware.
//   Static/DC removal is performed later in the PC analysis.
// - In FULL_DIAGNOSTIC mode, gyroscope zero-rate bias is estimated
//   while stationary and removed.
// - A binary configuration packet identifies the active acquisition
//   configuration and PCA channels.
// - Data packet payload size varies with both the number of detected
//   sensors AND the acquisition mode.
// - I2C error counters (NACK vs. timeout/short-read) are cumulative,
//   not per-sample, to avoid adding meaningful per-frame overhead -
//   they still let acquisition-health issues be diagnosed after a run.
//
// Binary frame:
//   sync0, sync1, packet_type, payload_size, payload..., checksum
//
// CONFIG payload:
//   firmware_version     [u8]   bump when protocol/config changes
//   dlpf_cfg              [u8]
//   smplrt_div             [u8]
//   acquisition_mode      [u8]   0 = FAST_ACCEL, 1 = FULL_DIAGNOSTIC
//   target_poll_rate_hz   [u32]
//   sensor_count          [u8]
//   active PCA channels   [u8 x sensor_count]
//
// DATA payload:
//   t_us                    [u32]
//   sample_index            [u32]
//   missed_deadlines_total  [u32]  cumulative, all-time
//   i2c_error_total         [u32]  cumulative: ANY I2C problem (bad
//                                  endTransmission status on any write,
//                                  OR a short/failed requestFrom read)
//   i2c_nack_total          [u32]  cumulative subset: NACK on address
//                                  or data - suggests a genuine bus/
//                                  wiring problem
//   i2c_timeout_total       [u32]  cumulative subset: timeout or short
//                                  read - more consistent with timing
//                                  overload than a hard wiring fault,
//                                  though this classification is an
//                                  engineering judgement call, not a
//                                  certainty
//   for each active sensor, in CONFIG order:
//     FAST_ACCEL mode:      ax ay az [int16 each]                (6 bytes)
//     FULL_DIAGNOSTIC mode: ax ay az gx gy gz [int16 each]       (12 bytes)
//
// Per-cycle timing diagnostics (cycle duration, scheduling lateness)
// and per-transaction I2C sub-timing are intentionally NOT transmitted
// here - they were used during acquisition-characterization validation
// (see main_validation.cpp), but the achieved sampling rate and jitter
// are fully derivable PC-side from consecutive t_us values (confirmed
// to agree with device-measured cycle duration to within ~1 us during
// validation), so transmitting them again in every production frame
// would be redundant per-sample overhead with no analytical benefit.
//
// All multibyte numeric fields are little-endian.
// ============================================================

// ============================================================
// ACQUISITION MODE - change this constant and reflash to switch modes.
// ============================================================
const uint8_t ACQ_MODE_FAST_ACCEL = 0;
const uint8_t ACQ_MODE_FULL_DIAGNOSTIC = 1;
const uint8_t ACQUISITION_MODE = ACQ_MODE_FAST_ACCEL;

// Firmware/protocol version. Bump any time DLPF_CFG, SMPLRT_DIV, the
// frame layout, or the acquisition-mode set changes, so a recorded
// dataset's .meta.json sidecar can always be traced back to the exact
// firmware that produced it.
const uint8_t FIRMWARE_VERSION = 7;

#define MPU_ADDR 0x68
#define PCA_ADDR 0x70

#define SDA_PIN 21
#define SCL_PIN 22

const uint32_t SERIAL_BAUD = 921600;

const uint32_t I2C_CLOCK_HZ = 400000;

// ESP32 output stream target. Validated for 2-3 sensors in FAST_ACCEL
// mode at 400 kHz I2C; if the sensor count or acquisition mode changes
// meaningfully, this should be re-validated with main_validation.cpp
// rather than assumed.
//
// 1,000,000 / SAMPLE_RATE_HZ is not necessarily an integer, so a
// rational microsecond scheduler is used below instead of truncating
// the period - this is generic to whatever SAMPLE_RATE_HZ is set to.
const uint32_t SAMPLE_RATE_HZ = 800;
const uint32_t SAMPLE_PERIOD_BASE_US = 1000000UL / SAMPLE_RATE_HZ;
const uint32_t SAMPLE_PERIOD_REMAINDER = 1000000UL % SAMPLE_RATE_HZ;

// MPU6050 register configuration.
const uint8_t DLPF_CFG = 0x01;       // ~184 Hz accel BW, ~188 Hz gyro BW
const uint8_t GYRO_CONFIG = 0x00;    // +/-250 deg/s
const uint8_t ACCEL_CONFIG = 0x10;   // +/-8 g
const uint8_t SMPLRT_DIV = 0x00;

// MPU6050 internal register-update rate, CALCULATED from the active
// DLPF_CFG/SMPLRT_DIV rather than hardcoded. This formula
// (gyro output rate / (1+SMPLRT_DIV)) is valid for DLPF_CFG 1-6, which
// is what this project always uses (DLPF_CFG=1 above); the 8 kHz base
// rate case (DLPF_CFG=0 or 7) is not handled here since it is not used.
const uint32_t MPU_GYRO_OUTPUT_RATE_HZ = 1000;
const uint32_t MPU_INTERNAL_RATE_HZ =
    MPU_GYRO_OUTPUT_RATE_HZ / (1 + SMPLRT_DIV);
const uint32_t MPU_INTERNAL_PERIOD_US = 1000000UL / MPU_INTERNAL_RATE_HZ;

// Computed, not hardcoded - the acquisition Nyquist frequency implied by
// the current SAMPLE_RATE_HZ. Printed at startup; also useful if this
// project ever wants to sanity-check the analysis-side Nyquist against
// the firmware's own target.
const float ESP32_OUTPUT_NYQUIST_HZ = SAMPLE_RATE_HZ / 2.0f;

// Maximum number of sensors/channels used by this project. The system
// does NOT assume this many sensors are actually present - detection is
// dynamic (see detectSensors()). This is only an upper bound on the
// number of PCA9548A channels scanned.
const int MAX_SENSORS = 3;
const uint8_t POSSIBLE_SENSOR_CHANNELS[MAX_SENSORS] = {0, 1, 2};

// Gyro zero-rate calibration samples (FULL_DIAGNOSTIC mode only).
// 500 samples at the MPU's actual internal rate (computed above, not
// hardcoded) covers roughly half a second at 1000 Hz; adjust if a
// longer calibration window is wanted.
const int CALIBRATION_SAMPLES = 500;

const float ACCEL_SCALE = 4096.0f;   // +/-8 g raw counts per g
const float GYRO_SCALE = 131.0f;     // +/-250 deg/s raw counts per deg/s

const uint8_t SYNC0 = 0xAA;
const uint8_t SYNC1 = 0x55;

const uint8_t PACKET_TYPE_DATA = 0x01;
const uint8_t PACKET_TYPE_CONFIG = 0x02;

const uint8_t BYTES_PER_SENSOR_FAST_ACCEL = 3 * 2;        // ax, ay, az
const uint8_t BYTES_PER_SENSOR_FULL_DIAGNOSTIC = 6 * 2;    // + gx, gy, gz
const uint8_t BYTES_PER_SENSOR =
    (ACQUISITION_MODE == ACQ_MODE_FULL_DIAGNOSTIC)
        ? BYTES_PER_SENSOR_FULL_DIAGNOSTIC
        : BYTES_PER_SENSOR_FAST_ACCEL;

// t_us + sample_index + missed_deadlines_total + i2c_error_total +
// i2c_nack_total + i2c_timeout_total
const uint8_t BASE_DATA_PAYLOAD_SIZE = 4 * 6;

const uint8_t MAX_PAYLOAD_SIZE = 64;

struct GyroOffset {
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

uint32_t streamStartUs = 0;
uint32_t sampleIndex = 0;
uint32_t missedDeadlines = 0;

// Cumulative I2C health counters. Reset to zero right before streaming
// starts (see setup()) so detection/setup-time I2C activity doesn't
// contaminate the run's reported error statistics.
uint32_t i2cErrorTotal = 0;
uint32_t i2cNackTotal = 0;
uint32_t i2cTimeoutTotal = 0;

GyroOffset gyroOffsets[MAX_SENSORS];
uint8_t activeChannels[MAX_SENSORS];
uint8_t activeSensorCount = 0;
uint8_t dataPayloadSize = BASE_DATA_PAYLOAD_SIZE;

// ============================================================
// I2C ERROR CLASSIFICATION
// ============================================================

// Wire.endTransmission() status codes (standard Arduino Wire semantics):
//   0 = success, 1 = data too long, 2 = NACK on address,
//   3 = NACK on data, 4 = other error, 5 = timeout (ESP32 core).
void recordI2cWriteStatus(uint8_t status) {
  if (status == 0) {
    return;
  }

  i2cErrorTotal++;

  if (status == 2 || status == 3) {
    i2cNackTotal++;
  } else if (status == 5) {
    i2cTimeoutTotal++;
  }
  // status 1 and 4 fold into i2cErrorTotal only - neither a clear
  // NACK-like bus fault nor clearly a timing symptom.
}

// A short read is treated as an I2C error, bucketed under the
// "timing-suggestive" counter as an engineering judgement call (short
// reads most commonly arise from bus contention under load), not a
// certainty - it could also stem from a transient electrical glitch.
void recordI2cReadResult(int expectedBytes, int actualBytes) {
  if (actualBytes == expectedBytes) {
    return;
  }

  i2cErrorTotal++;
  i2cTimeoutTotal++;
}

// ============================================================
// I2C / MPU HELPERS
// ============================================================

void selectPCAChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1U << channel);
  const uint8_t status = Wire.endTransmission(true);
  recordI2cWriteStatus(status);
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
  const uint8_t status = Wire.endTransmission(true);
  recordI2cWriteStatus(status);
  return status == 0;
}

// Full 14-byte accel+temperature+gyro burst read. Used only in
// ACQ_MODE_FULL_DIAGNOSTIC.
bool readMPU(
    int16_t &ax, int16_t &ay, int16_t &az,
    int16_t &gx, int16_t &gy, int16_t &gz) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  const uint8_t writeStatus = Wire.endTransmission(false);
  recordI2cWriteStatus(writeStatus);
  if (writeStatus != 0) {
    return false;
  }

  const int bytesRead = Wire.requestFrom((int)MPU_ADDR, 14, (int)true);
  recordI2cReadResult(14, bytesRead);

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

// Accelerometer-only 6-byte burst read (registers 0x3B-0x40, confirmed
// contiguous in the MPU6050 register map). Used only in
// ACQ_MODE_FAST_ACCEL - this is the normal mode for vibration analysis.
bool readAccelOnly(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  const uint8_t writeStatus = Wire.endTransmission(false);
  recordI2cWriteStatus(writeStatus);
  if (writeStatus != 0) {
    return false;
  }

  const int bytesRead = Wire.requestFrom((int)MPU_ADDR, 6, (int)true);
  recordI2cReadResult(6, bytesRead);

  if (bytesRead != 6) {
    return false;
  }

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());

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
  uint8_t frame[2 + 1 + 1 + MAX_PAYLOAD_SIZE + 1];
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

      delayMicroseconds(MPU_INTERNAL_PERIOD_US);
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
  uint8_t payload[1 + 1 + 1 + 1 + 4 + 1 + MAX_SENSORS];
  uint8_t idx = 0;

  appendU8(payload, idx, FIRMWARE_VERSION);
  appendU8(payload, idx, DLPF_CFG);
  appendU8(payload, idx, SMPLRT_DIV);
  appendU8(payload, idx, ACQUISITION_MODE);
  appendU32(payload, idx, SAMPLE_RATE_HZ);

  appendU8(payload, idx, activeSensorCount);
  for (int i = 0; i < activeSensorCount; i++) {
    appendU8(payload, idx, activeChannels[i]);
  }

  sendFrame(PACKET_TYPE_CONFIG, payload, idx);
}

void sendDataFrame() {
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint8_t idx = 0;

  appendU32(payload, idx, micros() - streamStartUs);
  appendU32(payload, idx, sampleIndex++);
  appendU32(payload, idx, missedDeadlines);
  appendU32(payload, idx, i2cErrorTotal);
  appendU32(payload, idx, i2cNackTotal);
  appendU32(payload, idx, i2cTimeoutTotal);

  for (int sensor = 0; sensor < activeSensorCount; sensor++) {
    selectPCAChannel(activeChannels[sensor]);

    int16_t rawAx = 0;
    int16_t rawAy = 0;
    int16_t rawAz = 0;

    if (ACQUISITION_MODE == ACQ_MODE_FULL_DIAGNOSTIC) {
      int16_t rawGx = 0;
      int16_t rawGy = 0;
      int16_t rawGz = 0;

      const bool ok =
          readMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

      // Acceleration is intentionally kept close to the sensor
      // measurement. Gravity/DC removal is performed in the PC analysis.
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
    } else {
      const bool ok = readAccelOnly(rawAx, rawAy, rawAz);

      const int16_t ax = ok ? rawAx : 0;
      const int16_t ay = ok ? rawAy : 0;
      const int16_t az = ok ? rawAz : 0;

      appendI16(payload, idx, ax);
      appendI16(payload, idx, ay);
      appendI16(payload, idx, az);
    }
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

  Serial.println("# ESP32 AUTO-DETECT MPU6050 PCA9548A binary logger - V7");
  Serial.print("# Acquisition mode: ");
  Serial.println(
      ACQUISITION_MODE == ACQ_MODE_FULL_DIAGNOSTIC
          ? "FULL_DIAGNOSTIC (accel+gyro+temp)"
          : "FAST_ACCEL (accelerometer-only)");
  Serial.print("# Baud: ");
  Serial.println(SERIAL_BAUD);
  Serial.print("# I2C clock: ");
  Serial.println(I2C_CLOCK_HZ);
  Serial.print("# Target sample rate: ");
  Serial.print(SAMPLE_RATE_HZ);
  Serial.println(" Hz");
  Serial.print("# Output Nyquist frequency (computed, SAMPLE_RATE_HZ/2): ");
  Serial.print(ESP32_OUTPUT_NYQUIST_HZ);
  Serial.println(" Hz");
  Serial.print("# MPU6050 DLPF_CFG: ");
  Serial.println(DLPF_CFG);
  Serial.print("# MPU6050 SMPLRT_DIV: ");
  Serial.println(SMPLRT_DIV);
  Serial.print("# MPU6050 internal register-update rate (computed): ");
  Serial.print(MPU_INTERNAL_RATE_HZ);
  Serial.println(" Hz");
  Serial.println("# Accel bandwidth (DLPF_CFG=1): approximately 184 Hz");
  Serial.println("# Gyro bandwidth (DLPF_CFG=1): approximately 188 Hz");
  Serial.println("# Accelerometer range: +/-8 g");

  if (ACQUISITION_MODE == ACQ_MODE_FULL_DIAGNOSTIC) {
    Serial.println("# Gyroscope range: +/-250 deg/s");
  }

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

  if (ACQUISITION_MODE == ACQ_MODE_FULL_DIAGNOSTIC) {
    calibrateGyros();
  } else {
    Serial.println(
        "# FAST_ACCEL mode: gyro calibration skipped "
        "(no gyro data acquired in this mode).");
  }

  // Reset run-scoped counters immediately before streaming starts, so
  // detection/setup-time activity doesn't contaminate the run's
  // reported statistics.
  sampleIndex = 0;
  missedDeadlines = 0;
  i2cErrorTotal = 0;
  i2cNackTotal = 0;
  i2cTimeoutTotal = 0;
  streamStartUs = micros();

  Serial.println("# Binary stream starting now.");
  Serial.println("START_BINARY");
  Serial.flush();

  // Send the active acquisition configuration before normal data packets.
  sendConfigurationFrame();
}

void loop() {
  // Rational microsecond scheduler, generic to whatever SAMPLE_RATE_HZ is
  // currently set to. Distributes the fractional remainder across
  // successive samples so the long-term average rate is exact, without
  // truncating the period to an integer microsecond value.
  static uint32_t nextSampleUs = micros();
  static uint32_t remainderAccumulator = 0;

  const uint32_t now = micros();

  if ((int32_t)(now - nextSampleUs) >= 0) {
    const uint32_t lateness = now - nextSampleUs;

    // If execution is badly delayed, count the event and re-anchor the
    // scheduler. This avoids rapid "catch-up" packets that would corrupt
    // the effective sample spacing.
    if (lateness >= (SAMPLE_PERIOD_BASE_US + 1U)) {
      missedDeadlines++;
      nextSampleUs = now;
      remainderAccumulator = 0;
    }

    sendDataFrame();

    // Add one nominal period.
    nextSampleUs += SAMPLE_PERIOD_BASE_US;

    // Distribute the fractional remainder across successive samples.
    remainderAccumulator += SAMPLE_PERIOD_REMAINDER;
    if (remainderAccumulator >= SAMPLE_RATE_HZ) {
      nextSampleUs += 1U;
      remainderAccumulator -= SAMPLE_RATE_HZ;
    }
  }
}
