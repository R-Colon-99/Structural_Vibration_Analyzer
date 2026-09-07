// VALIDATION BUILD - Acquisition characterization firmware
// Structural Vibration Analyzer
//
// PURPOSE:
// Temporary validation version of the acquisition firmware, used to
// experimentally determine the maximum stable 3-sensor polling rate, real
// sample-interval jitter, and whether polling below the MPU6050's internal
// update rate produces genuinely fresh (non-duplicate) samples. This
// answers Tests 1-3 of the validation plan only (throughput, timing/jitter,
// repeated-sample behavior). DLPF frequency-response, aliasing, and
// multi-sensor synchronization tests are NOT implemented by this build.
//
// This is NOT the production firmware. The original full accel+gyro+temp
// logger (main.cpp) is untouched and remains the file to fly for normal
// operation.
//
// KEY DIFFERENCES FROM main.cpp:
//   - MPU6050 configured DLPF_CFG=1, SMPLRT_DIV=0 (1000 Hz internal
//     accel/gyro register-update rate), instead of DLPF_CFG=2/SMPLRT_DIV=3
//     (250 Hz).
//   - Accelerometer-ONLY burst read: 6 bytes from registers 0x3B-0x40
//     (ACCEL_XOUT_H/L, ACCEL_YOUT_H/L, ACCEL_ZOUT_H/L - registers 59-64),
//     confirmed contiguous in the MPU6050 register map. No temperature or
//     gyro registers are read during acquisition.
//   - Gyro calibration is skipped at RUNTIME (this build never reads gyro
//     data during streaming). The original calibration routine is
//     preserved verbatim below inside an #if 0 block, unchanged, so
//     restoring full-mode acquisition later does not require rewriting it
//     - just move it back out of the #if 0 guard and call it again from
//     setup(). See "FULL-MODE RESTORE PATH" below.
//   - SAMPLE_RATE_HZ is a single constant you change and reflash to sweep
//     candidate polling rates. Do not assume any one value is correct -
//     that is the entire point of this validation build.
//   - Every DATA frame carries per-cycle timing diagnostics (cycle
//     duration, scheduling lateness, cumulative missed-deadline counter)
//     PLUS cumulative I2C error/short-read/NACK/timeout counters, so bus
//     communication problems can be told apart from pure scheduling
//     overload after the fact.
//   - Optional per-transaction I2C sub-timing (PCA channel-select time,
//     MPU6050 read time) is gated behind I2C_TIMING_INSTRUMENTATION.
//     Leave at 0 for throughput/jitter runs (Tests 1-2); set to 1 only
//     when specifically comparing measured bus sub-timings against the
//     theoretical ~50 us (PCA select) / ~255 us (6-byte accel read)
//     estimates.
//   - CONFIG frame now also reports DLPF_CFG, SMPLRT_DIV, acquisition
//     mode, and a firmware build identifier, so any recorded dataset can
//     always be traced back to the exact protocol/configuration that
//     produced it.
//
// FULL-MODE RESTORE PATH (when validation work is done):
//   1. Move calibrateGyros() (below, inside #if 0) back into an active
//      code path and call it from setup() before streaming starts, as
//      main.cpp does.
//   2. Switch the per-sensor read in sendDataFrame() from readAccelOnly()
//      to readMPU_FullAccelTempGyro() (retained below, unused).
//   3. Increase BYTES_PER_SENSOR from 6 to 12 and change
//      acquisitionMode from ACQ_MODE_ACCEL_ONLY to
//      ACQ_MODE_FULL_ACCEL_GYRO_TEMP.
//   None of this requires touching the scheduler, framing, or error-
//   counting logic - those are acquisition-mode-agnostic by design.
//
// Binary frame format (unchanged sync/checksum scheme from main.cpp):
//   sync0, sync1, packet_type, payload_size, payload..., checksum
//
// CONFIG payload:
//   protocol_version        [u8]   bump if this frame layout changes
//   build_id                [u32]  manually incremented per firmware
//                                  configuration change (see
//                                  FIRMWARE_BUILD_ID below)
//   build_tag               [8 bytes ASCII, zero-padded]
//   dlpf_cfg                [u8]
//   smplrt_div              [u8]
//   acquisition_mode        [u8]   0 = accelerometer-only (this build)
//                                  1 = full accel+gyro+temp (reserved)
//   instrumentation_enabled [u8]   0 or 1
//   target_poll_rate_hz     [u32]
//   sensor_count            [u8]
//   active PCA channels     [u8 x sensor_count]
//
// DATA payload:
//   t_us                    [u32]
//   sample_index            [u32]
//   cycle_duration_us       [u32]  wall-clock time since previous DATA frame
//   lateness_us             [u32]  how late this cycle fired vs. its
//                                  scheduled deadline; 0 = on time/early
//   missed_deadlines_total  [u32]  cumulative, all-time
//   i2c_error_total         [u32]  cumulative: ANY I2C problem (bad
//                                  endTransmission status on any write,
//                                  OR a short/failed requestFrom read)
//   i2c_nack_total          [u32]  cumulative subset: endTransmission
//                                  returned 2 (NACK on address) or 3
//                                  (NACK on data) - suggests a genuine
//                                  bus/wiring/address problem rather than
//                                  a timing issue
//   i2c_timeout_total       [u32]  cumulative subset: endTransmission
//                                  returned 5 (timeout, on cores that
//                                  support it), OR requestFrom returned
//                                  fewer bytes than requested - these are
//                                  the categories most consistent with
//                                  bus contention / timing overload
//                                  rather than a hard wiring fault, but
//                                  this classification is an engineering
//                                  judgement call, not a certainty - a
//                                  short read COULD also be caused by a
//                                  transient electrical glitch unrelated
//                                  to timing. Treat i2c_nack_total as the
//                                  stronger "real bus problem" signal and
//                                  i2c_timeout_total as the weaker,
//                                  timing-suggestive one.
//   for each active sensor, in CONFIG order:
//       ax ay az [int16 each]
//   (only if I2C_TIMING_INSTRUMENTATION==1, appended AFTER the ax/ay/az
//    of ALL sensors above, one entry per sensor in CONFIG order:)
//       pca_select_us [u16]
//       mpu_read_us   [u16]
//   ("total time per sensor" and "total complete multi-sensor scan time"
//    are intentionally NOT transmitted as separate fields - they are
//    simple sums of pca_select_us+mpu_read_us, derived downstream in the
//    analysis script, to avoid redundant wire bytes.)
//
// All multibyte numeric fields are little-endian.
// ============================================================

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// FIRMWARE / PROTOCOL IDENTIFICATION
// Bump BOTH of these any time DLPF_CFG, SMPLRT_DIV, the read mode, or the
// frame layout changes, so recorded datasets can always be traced back to
// the exact configuration that produced them.
// ============================================================
const uint8_t PROTOCOL_VERSION = 1;
const uint32_t FIRMWARE_BUILD_ID = 1;
const char BUILD_TAG[8] = "VAL-A01";  // "VALidation, Accel-only, rev 01"

// ============================================================
// VALIDATION SWEEP PARAMETER - change this and reflash to test
// candidate rates. Do not assume any one value is correct; this is
// exactly what Tests 1-2 are for.
//
// Candidate values to test: 400, 500, 600, 700, 800, 900, 1000 Hz
// (and beyond, if a rate is found to be comfortably stable and you want
// to find where instability actually begins).
// ============================================================
const uint32_t SAMPLE_RATE_HZ = 800;

// Set to 1 only when deliberately measuring I2C sub-transaction timing.
// Leave at 0 for throughput (Test 1) and jitter (Test 2) characterization
// runs, since the extra micros() calls add overhead to the hot path.
#define I2C_TIMING_INSTRUMENTATION 0

#define MPU_ADDR 0x68
#define PCA_ADDR 0x70

#define SDA_PIN 21
#define SCL_PIN 22

const uint32_t SERIAL_BAUD = 921600;
const uint32_t I2C_CLOCK_HZ = 400000;

// Rational microsecond scheduler: 1,000,000 / SAMPLE_RATE_HZ is not
// necessarily an integer, so the fractional remainder is distributed
// across successive samples rather than truncated, to avoid long-term
// drift. Generic to any SAMPLE_RATE_HZ.
const uint32_t SAMPLE_PERIOD_BASE_US = 1000000UL / SAMPLE_RATE_HZ;
const uint32_t SAMPLE_PERIOD_REMAINDER = 1000000UL % SAMPLE_RATE_HZ;

// Maximum number of sensors/channels used by this project. The logger
// and analysis scripts read the ACTUAL detected count from the CONFIG
// frame at runtime and must not assume this maximum was reached.
const int MAX_SENSORS = 3;
const uint8_t POSSIBLE_SENSOR_CHANNELS[MAX_SENSORS] = {0, 1, 2};

// MPU6050 register configuration for this validation build.
// DLPF_CFG=1 -> accel BW ~184 Hz, gyro BW ~188 Hz, internal Fs = 1 kHz.
// SMPLRT_DIV=0 -> Sample Rate = 1000 Hz / (1 + 0) = 1000 Hz register
// update rate. This matches the accelerometer's own fixed 1 kHz native
// update rate exactly (MPU6050 Register 25 documentation), so there is no
// combination of DLPF_CFG/SMPLRT_DIV here that produces duplicate
// accelerometer register values on the MPU side itself - any repeats seen
// at the ESP32's polling rate would have to come from polling faster than
// this 1000 Hz internal rate (see Test 3 / analyze_validation.py).
const uint8_t DLPF_CFG = 0x01;
const uint8_t GYRO_CONFIG = 0x00;    // +/-250 deg/s (kept configured but
                                      // unused/unread in this build, so a
                                      // future full-mode restore doesn't
                                      // need to touch sensor setup).
const uint8_t ACCEL_CONFIG = 0x10;   // +/-8 g (unchanged from main.cpp)
const uint8_t SMPLRT_DIV = 0x00;

const float ACCEL_SCALE = 4096.0f;   // +/-8 g raw counts per g

const uint8_t SYNC0 = 0xAA;
const uint8_t SYNC1 = 0x55;

const uint8_t PACKET_TYPE_DATA = 0x01;
const uint8_t PACKET_TYPE_CONFIG = 0x02;

const uint8_t ACQ_MODE_ACCEL_ONLY = 0;
const uint8_t ACQ_MODE_FULL_ACCEL_GYRO_TEMP = 1;  // reserved for restore
const uint8_t ACQUISITION_MODE = ACQ_MODE_ACCEL_ONLY;

// Accelerometer-only: 3 x int16 = 6 bytes per sensor.
const uint8_t BYTES_PER_SENSOR = 3 * 2;

#if I2C_TIMING_INSTRUMENTATION
const uint8_t INSTRUMENTATION_BYTES_PER_SENSOR = 2 + 2; // pca_select_us + mpu_read_us
#else
const uint8_t INSTRUMENTATION_BYTES_PER_SENSOR = 0;
#endif

// t_us + sample_index + cycle_duration_us + lateness_us +
// missed_deadlines_total + i2c_error_total + i2c_nack_total +
// i2c_timeout_total
const uint8_t BASE_DATA_PAYLOAD_SIZE = 4 * 8;

const uint8_t MAX_PAYLOAD_SIZE = 96;

uint32_t streamStartUs = 0;
uint32_t sampleIndex = 0;
uint32_t missedDeadlines = 0;

// Cumulative I2C health counters. Reset to zero right before streaming
// starts (see setup()) so setup-time/detection-time I2C activity doesn't
// contaminate the run's error statistics.
uint32_t i2cErrorTotal = 0;
uint32_t i2cNackTotal = 0;
uint32_t i2cTimeoutTotal = 0;

uint8_t activeChannels[MAX_SENSORS];
uint8_t activeSensorCount = 0;
uint8_t dataPayloadSize = BASE_DATA_PAYLOAD_SIZE;

// ============================================================
// I2C ERROR CLASSIFICATION
// ============================================================

// Wire.endTransmission() status codes (standard Arduino Wire semantics):
//   0 = success
//   1 = data too long to fit in transmit buffer
//   2 = received NACK on transmit of address
//   3 = received NACK on transmit of data
//   4 = other error
//   5 = timeout (supported on ESP32 Arduino core)
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
  // status 1 (data too long) and 4 (other) fold into i2cErrorTotal only -
  // neither a NACK-like bus fault nor clearly a timing symptom.
}

// A short read (fewer bytes returned than requested) is treated as an
// I2C error, and - as an engineering judgement call, not a certainty -
// bucketed under the "timing-suggestive" counter, since it most commonly
// arises from bus contention under load rather than a hard wiring fault.
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

// Retained, UNUSED in this accel-only validation build. Original full
// 14-byte accel+temp+gyro burst read from main.cpp, kept so restoring a
// full diagnostic acquisition mode later is a small, clearly-delineated
// change (see "FULL-MODE RESTORE PATH" above) rather than a rewrite.
bool readMPU_FullAccelTempGyro(
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

// Accelerometer-only burst read: 6 bytes from registers 0x3B-0x40
// (ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H,
// ACCEL_ZOUT_L - registers 59-64, confirmed contiguous in the MPU6050
// register map). No temperature or gyro registers are touched.
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
// FULL-MODE GYRO CALIBRATION - PRESERVED, NOT ACTIVE
// This is main.cpp's original calibrateGyros(), unchanged, kept inside
// #if 0 so it compiles out of this validation build but is trivially
// restorable: delete the #if 0 / #endif guard, declare a GyroOffset
// struct + gyroOffsets[MAX_SENSORS] array as in main.cpp, and call this
// from setup() before streaming starts.
// ============================================================
#if 0
void calibrateGyros() {
  Serial.println("# Calibrating gyroscopes (keep sensors stationary)...");

  const int CALIBRATION_SAMPLES = 500;
  const uint32_t MPU_INTERNAL_RATE_HZ = 1000; // matches SMPLRT_DIV=0 above
  const uint32_t MPU_INTERNAL_PERIOD_US = 1000000UL / MPU_INTERNAL_RATE_HZ;

  for (int i = 0; i < activeSensorCount; i++) {
    long sumGx = 0, sumGy = 0, sumGz = 0;

    selectPCAChannel(activeChannels[i]);

    for (int s = 0; s < CALIBRATION_SAMPLES; s++) {
      int16_t ax, ay, az, gx, gy, gz;
      if (readMPU_FullAccelTempGyro(ax, ay, az, gx, gy, gz)) {
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
      }
      delayMicroseconds(MPU_INTERNAL_PERIOD_US);
    }

    gyroOffsets[i].gx = (float)sumGx / CALIBRATION_SAMPLES;
    gyroOffsets[i].gy = (float)sumGy / CALIBRATION_SAMPLES;
    gyroOffsets[i].gz = (float)sumGz / CALIBRATION_SAMPLES;
  }

  Serial.println("# Gyro calibration complete.");
}
#endif

// ============================================================
// PACKET HELPERS
// ============================================================

void appendU8(uint8_t *buffer, uint8_t &index, uint8_t value) {
  buffer[index++] = value;
}

void appendU16(uint8_t *buffer, uint8_t &index, uint16_t value) {
  buffer[index++] = (uint8_t)(value & 0xFF);
  buffer[index++] = (uint8_t)((value >> 8) & 0xFF);
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
// SENSOR DETECTION (gyro calibration skipped - not needed, accel-only)
// ============================================================

void detectSensors() {
  activeSensorCount = 0;

  Serial.println("# VALIDATION BUILD - Scanning PCA9548A channels for MPU6050 sensors...");

  for (int i = 0; i < MAX_SENSORS; i++) {
    const uint8_t channel = POSSIBLE_SENSOR_CHANNELS[i];

    selectPCAChannel(channel);
    delay(5);

    Serial.print("# Checking PCA channel ");
    Serial.print(channel);
    Serial.print(": ");

    if (mpuPresentOnSelectedChannel()) {
      if (setupMPUOnSelectedChannel()) {
        Serial.println("MPU6050 found and configured (DLPF_CFG=1, SMPLRT_DIV=0)");
        activeChannels[activeSensorCount++] = channel;
      } else {
        Serial.println("MPU6050 found but configuration failed");
      }
    } else {
      Serial.println("NOT FOUND");
    }
  }

  dataPayloadSize = BASE_DATA_PAYLOAD_SIZE
      + (activeSensorCount * BYTES_PER_SENSOR)
      + (activeSensorCount * INSTRUMENTATION_BYTES_PER_SENSOR);

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

  Serial.print("# Target poll rate (Hz): ");
  Serial.println(SAMPLE_RATE_HZ);

  Serial.print("# I2C timing instrumentation: ");
  Serial.println(I2C_TIMING_INSTRUMENTATION ? "ENABLED" : "disabled");

  Serial.print("# Binary data payload size: ");
  Serial.println(dataPayloadSize);
}

// ============================================================
// CONFIG + DATA STREAM
// ============================================================

void sendConfigurationFrame() {
  uint8_t payload[1 + 4 + 8 + 1 + 1 + 1 + 1 + 4 + 1 + MAX_SENSORS];
  uint8_t idx = 0;

  appendU8(payload, idx, PROTOCOL_VERSION);
  appendU32(payload, idx, FIRMWARE_BUILD_ID);

  for (int i = 0; i < 8; i++) {
    appendU8(payload, idx, (uint8_t)BUILD_TAG[i]);
  }

  appendU8(payload, idx, DLPF_CFG);
  appendU8(payload, idx, SMPLRT_DIV);
  appendU8(payload, idx, ACQUISITION_MODE);
  appendU8(payload, idx, I2C_TIMING_INSTRUMENTATION ? 1 : 0);
  appendU32(payload, idx, SAMPLE_RATE_HZ);

  appendU8(payload, idx, activeSensorCount);
  for (int i = 0; i < activeSensorCount; i++) {
    appendU8(payload, idx, activeChannels[i]);
  }

  sendFrame(PACKET_TYPE_CONFIG, payload, idx);
}

void sendDataFrame(uint32_t cycleDurationUs, uint32_t latenessUs) {
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint8_t idx = 0;

  appendU32(payload, idx, micros() - streamStartUs);
  appendU32(payload, idx, sampleIndex++);
  appendU32(payload, idx, cycleDurationUs);
  appendU32(payload, idx, latenessUs);
  appendU32(payload, idx, missedDeadlines);
  appendU32(payload, idx, i2cErrorTotal);
  appendU32(payload, idx, i2cNackTotal);
  appendU32(payload, idx, i2cTimeoutTotal);

#if I2C_TIMING_INSTRUMENTATION
  uint16_t pcaSelectUs[MAX_SENSORS] = {0};
  uint16_t mpuReadUs[MAX_SENSORS] = {0};
#endif

  for (int sensor = 0; sensor < activeSensorCount; sensor++) {
    int16_t rawAx = 0;
    int16_t rawAy = 0;
    int16_t rawAz = 0;

#if I2C_TIMING_INSTRUMENTATION
    const uint32_t tSelStart = micros();
    selectPCAChannel(activeChannels[sensor]);
    const uint32_t tSelEnd = micros();
    pcaSelectUs[sensor] = (uint16_t)(tSelEnd - tSelStart);

    const uint32_t tReadStart = micros();
    const bool ok = readAccelOnly(rawAx, rawAy, rawAz);
    const uint32_t tReadEnd = micros();
    mpuReadUs[sensor] = (uint16_t)(tReadEnd - tReadStart);
#else
    selectPCAChannel(activeChannels[sensor]);
    const bool ok = readAccelOnly(rawAx, rawAy, rawAz);
#endif

    const int16_t ax = ok ? rawAx : 0;
    const int16_t ay = ok ? rawAy : 0;
    const int16_t az = ok ? rawAz : 0;

    appendI16(payload, idx, ax);
    appendI16(payload, idx, ay);
    appendI16(payload, idx, az);
  }

#if I2C_TIMING_INSTRUMENTATION
  for (int sensor = 0; sensor < activeSensorCount; sensor++) {
    appendU16(payload, idx, pcaSelectUs[sensor]);
    appendU16(payload, idx, mpuReadUs[sensor]);
  }
#endif

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

  Serial.println("# ESP32 ACQUISITION VALIDATION BUILD - accel-only, configurable rate");
  Serial.print("# Build tag: ");
  Serial.println(BUILD_TAG);
  Serial.print("# Build ID: ");
  Serial.println(FIRMWARE_BUILD_ID);
  Serial.print("# Protocol version: ");
  Serial.println(PROTOCOL_VERSION);
  Serial.print("# Baud: ");
  Serial.println(SERIAL_BAUD);
  Serial.print("# I2C clock: ");
  Serial.println(I2C_CLOCK_HZ);
  Serial.print("# MPU6050 DLPF_CFG: ");
  Serial.println(DLPF_CFG);
  Serial.print("# MPU6050 SMPLRT_DIV: ");
  Serial.println(SMPLRT_DIV);
  Serial.println("# MPU6050 internal register-update rate: 1000 Hz (fixed by DLPF_CFG=1/SMPLRT_DIV=0)");
  Serial.println("# Accelerometer bandwidth (DLPF_CFG=1): approximately 184 Hz");
  Serial.println("# Accelerometer range: +/-8 g");
  Serial.println("# Acquisition mode: ACCELEROMETER-ONLY (no gyro, no temperature)");

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

  // Reset run-scoped counters immediately before streaming starts, so
  // detection/setup-time I2C activity doesn't contaminate the run's
  // reported error statistics.
  sampleIndex = 0;
  missedDeadlines = 0;
  i2cErrorTotal = 0;
  i2cNackTotal = 0;
  i2cTimeoutTotal = 0;
  streamStartUs = micros();

  Serial.println("# Binary stream starting now.");
  Serial.println("START_BINARY");
  Serial.flush();

  sendConfigurationFrame();
}

void loop() {
  // Rational microsecond scheduler, generic to whatever SAMPLE_RATE_HZ is
  // currently set to. Distributes the fractional remainder across
  // successive samples so the long-term average rate is exact, without
  // truncating the period to an integer microsecond value.
  static uint32_t nextSampleUs = micros();
  static uint32_t remainderAccumulator = 0;
  static uint32_t previousCycleUs = 0;
  static bool firstCycle = true;

  const uint32_t now = micros();

  if ((int32_t)(now - nextSampleUs) >= 0) {
    uint32_t latenessUs = now - nextSampleUs;

    // If execution is badly delayed, count the event and re-anchor the
    // scheduler to avoid rapid "catch-up" packets that would corrupt the
    // effective sample spacing. The raw latenessUs value for THIS cycle
    // is still reported truthfully below, even when a re-anchor happens.
    if (latenessUs >= (SAMPLE_PERIOD_BASE_US + 1U)) {
      missedDeadlines++;
      nextSampleUs = now;
      remainderAccumulator = 0;
    }

    const uint32_t cycleDurationUs =
        firstCycle ? 0 : (now - previousCycleUs);
    previousCycleUs = now;
    firstCycle = false;

    sendDataFrame(cycleDurationUs, latenessUs);

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
