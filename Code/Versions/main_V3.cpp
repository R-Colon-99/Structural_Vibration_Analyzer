#include <Arduino.h>
#include <Wire.h>

// ============================================================
// ESP32 + PCA9548A + AUTO-DETECTED MPU6050 BINARY LOGGER
// ============================================================
// This firmware scans PCA9548A channels 0, 1, and 2.
// It only calibrates and streams data from sensors that are found.
//
// IMPORTANT:
// The PC logger must be able to handle variable payload sizes.
// Payload size = 8 + (detected_sensors * 12)
// 8 bytes = t_us + sample_index
// 12 bytes per sensor = ax ay az gx gy gz, each int16
// ============================================================

#define MPU_ADDR 0x68
#define PCA_ADDR 0x70

#define SDA_PIN 21
#define SCL_PIN 22

const uint32_t SERIAL_BAUD = 921600;

const float SAMPLE_RATE_HZ = 800.0;
const uint32_t SAMPLE_PERIOD_US = (uint32_t)(1000000.0 / SAMPLE_RATE_HZ);

// Maximum number of sensors/channels to scan.
const int MAX_SENSORS = 3;
const uint8_t POSSIBLE_SENSOR_CHANNELS[MAX_SENSORS] = {0, 1, 2};

const int CALIBRATION_SAMPLES = 1000;

const float ACCEL_SCALE = 4096.0; // ±8g raw counts per g
const float GYRO_SCALE = 131.0;   // ±250 deg/s raw counts per deg/s

// Binary packet format:
// sync0, sync1, packet_type, payload_size, t_us, sample_index,
// active sensor 1 ax ay az gx gy gz,
// active sensor 2 ax ay az gx gy gz,
// active sensor 3 ax ay az gx gy gz,
// checksum
//
// Only detected sensors are included.
// All numeric fields are little-endian.

const uint8_t SYNC0 = 0xAA;
const uint8_t SYNC1 = 0x55;
const uint8_t PACKET_TYPE_DATA = 0x01;
const uint8_t BYTES_PER_SENSOR = 6 * 2;
const uint8_t BASE_PAYLOAD_SIZE = 4 + 4; // t_us + sample_index

struct SensorOffset {
  float ax = 0;
  float ay = 0;
  float az = 0;
  float gx = 0;
  float gy = 0;
  float gz = 0;
};

uint32_t streamStartUs = 0;
uint32_t sampleIndex = 0;

SensorOffset offsets[MAX_SENSORS];
uint8_t activeChannels[MAX_SENSORS];
uint8_t activeSensorCount = 0;
uint8_t payloadSize = BASE_PAYLOAD_SIZE;

void selectPCAChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission(true);
}

void disablePCAChannels() {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void writeMPU(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  int bytesRead = Wire.requestFrom((int)MPU_ADDR, 14, (int)true);

  if (bytesRead != 14) {
    return false;
  }

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());

  Wire.read();
  Wire.read(); // temperature ignored

  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());

  return true;
}

bool mpuPresentOnSelectedChannel() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission(true) == 0;
}

void setupMPUOnSelectedChannel() {
  writeMPU(0x6B, 0x00); // wake up
  delay(100);

  writeMPU(0x1A, 0x01); // DLPF config, approx. 44 Hz accel bandwidth
  writeMPU(0x1B, 0x00); // gyro ±250 deg/s
  writeMPU(0x1C, 0x10); // accel ±8g
  writeMPU(0x19, 0x00); // sample rate divider
}

int16_t clampToInt16(float value) {
  if (value > 32767.0) return 32767;
  if (value < -32768.0) return -32768;
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

void detectSensors() {
  activeSensorCount = 0;

  Serial.println("# Scanning PCA9548A channels for MPU6050 sensors...");

  for (int i = 0; i < MAX_SENSORS; i++) {
    uint8_t channel = POSSIBLE_SENSOR_CHANNELS[i];
    selectPCAChannel(channel);
    delay(10);

    Serial.print("# Checking PCA channel ");
    Serial.print(channel);
    Serial.print(": ");

    if (mpuPresentOnSelectedChannel()) {
      Serial.println("MPU6050 found");

      setupMPUOnSelectedChannel();

      activeChannels[activeSensorCount] = channel;
      activeSensorCount++;
    } else {
      Serial.println("NOT FOUND");
    }
  }

  payloadSize = BASE_PAYLOAD_SIZE + (activeSensorCount * BYTES_PER_SENSOR);

  Serial.print("# Detected sensor count: ");
  Serial.println(activeSensorCount);

  Serial.print("# Active PCA channels: ");
  if (activeSensorCount == 0) {
    Serial.println("none");
  } else {
    for (int i = 0; i < activeSensorCount; i++) {
      Serial.print(activeChannels[i]);
      if (i < activeSensorCount - 1) Serial.print(", ");
    }
    Serial.println();
  }

  Serial.print("# Binary payload size: ");
  Serial.println(payloadSize);
}

void calibrateSensors() {
  Serial.print("# Calibrating ");
  Serial.print(activeSensorCount);
  Serial.println(" MPU6050 sensor(s). Keep all sensors still.");

  for (int sensor = 0; sensor < activeSensorCount; sensor++) {
    selectPCAChannel(activeChannels[sensor]);

    long axSum = 0, aySum = 0, azSum = 0;
    long gxSum = 0, gySum = 0, gzSum = 0;
    int validSamples = 0;

    int16_t ax, ay, az, gx, gy, gz;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      if (readMPU(ax, ay, az, gx, gy, gz)) {
        axSum += ax;
        aySum += ay - 4096; // assumes sensor Y axis is vertical during calibration
        azSum += az;

        gxSum += gx;
        gySum += gy;
        gzSum += gz;

        validSamples++;
      }

      delay(2);
    }

    if (validSamples == 0) {
      Serial.print("# WARNING: No valid calibration samples for active sensor ");
      Serial.print(sensor + 1);
      Serial.print(" on PCA channel ");
      Serial.println(activeChannels[sensor]);
      continue;
    }

    offsets[sensor].ax = axSum / (float)validSamples;
    offsets[sensor].ay = aySum / (float)validSamples;
    offsets[sensor].az = azSum / (float)validSamples;
    offsets[sensor].gx = gxSum / (float)validSamples;
    offsets[sensor].gy = gySum / (float)validSamples;
    offsets[sensor].gz = gzSum / (float)validSamples;

    Serial.print("# Active sensor ");
    Serial.print(sensor + 1);
    Serial.print(" on PCA channel ");
    Serial.print(activeChannels[sensor]);
    Serial.println(" calibration complete.");
  }

  Serial.println("# Calibration complete.");
}

void sendBinaryFrame() {
  const uint8_t frameSize = 2 + 1 + 1 + payloadSize + 1;
  uint8_t frame[2 + 1 + 1 + BASE_PAYLOAD_SIZE + (MAX_SENSORS * BYTES_PER_SENSOR) + 1];
  uint8_t idx = 0;

  appendU8(frame, idx, SYNC0);
  appendU8(frame, idx, SYNC1);
  appendU8(frame, idx, PACKET_TYPE_DATA);
  appendU8(frame, idx, payloadSize);
  appendU32(frame, idx, micros() - streamStartUs);
  appendU32(frame, idx, sampleIndex++);

  for (int sensor = 0; sensor < activeSensorCount; sensor++) {
    selectPCAChannel(activeChannels[sensor]);

    int16_t rawAx = 0, rawAy = 0, rawAz = 0;
    int16_t rawGx = 0, rawGy = 0, rawGz = 0;

    bool ok = readMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    int16_t ax = ok ? clampToInt16(rawAx - offsets[sensor].ax) : 0;
    int16_t ay = ok ? clampToInt16(rawAy - offsets[sensor].ay) : 0;
    int16_t az = ok ? clampToInt16(rawAz - offsets[sensor].az) : 0;
    int16_t gx = ok ? clampToInt16(rawGx - offsets[sensor].gx) : 0;
    int16_t gy = ok ? clampToInt16(rawGy - offsets[sensor].gy) : 0;
    int16_t gz = ok ? clampToInt16(rawGz - offsets[sensor].gz) : 0;

    appendI16(frame, idx, ax);
    appendI16(frame, idx, ay);
    appendI16(frame, idx, az);
    appendI16(frame, idx, gx);
    appendI16(frame, idx, gy);
    appendI16(frame, idx, gz);
  }

  frame[idx++] = calculateChecksum(frame, idx);
  Serial.write(frame, frameSize);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(800000);

  Serial.println("# ESP32 AUTO-DETECT MPU6050 PCA9548A binary logger");
  Serial.print("# Baud: ");
  Serial.println(SERIAL_BAUD);
  Serial.print("# Target sample rate: ");
  Serial.println(SAMPLE_RATE_HZ, 2);

  detectSensors();

  if (activeSensorCount == 0) {
    Serial.println("# ERROR: No MPU6050 sensors detected. Check wiring, power, and PCA9548A channels.");
    Serial.println("# Firmware halted. Reset the ESP32 after fixing the wiring.");
    disablePCAChannels();
    while (true) {
      delay(1000);
    }
  }

  calibrateSensors();

  sampleIndex = 0;
  streamStartUs = micros();

  Serial.println("# Binary stream starting now.");
  Serial.println("START_BINARY");
  Serial.flush();
}

void loop() {
  static uint32_t lastSample = micros();
  uint32_t now = micros();

  if ((uint32_t)(now - lastSample) >= SAMPLE_PERIOD_US) {
    lastSample += SAMPLE_PERIOD_US;
    sendBinaryFrame();
  }
}
