#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

#define SDA_PIN 21
#define SCL_PIN 22

const uint32_t SERIAL_BAUD = 921600;
const float SAMPLE_RATE_HZ = 500.0;
const uint32_t SAMPLE_PERIOD_US = 1000000.0 / SAMPLE_RATE_HZ;

const int CALIBRATION_SAMPLES = 1000;

float axOffset = 0, ayOffset = 0, azOffset = 0;
float gxOffset = 0, gyOffset = 0, gzOffset = 0;

const float ACCEL_SCALE = 16384.0; // ±2g
const float GYRO_SCALE = 131.0;    // ±250 deg/s

void writeMPU(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // temperature ignored

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void setupMPU() {
  writeMPU(0x6B, 0x00); // wake up
  delay(100);

  writeMPU(0x1A, 0x03); // DLPF config, ~44 Hz accel bandwidth
  writeMPU(0x1B, 0x00); // gyro ±250 deg/s
  writeMPU(0x1C, 0x00); // accel ±2g
  writeMPU(0x19, 0x01); // sample rate divider
}

void calibrateMPU() {
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  int16_t ax, ay, az, gx, gy, gz;

  Serial.println("# Calibrating MPU6050. Keep sensor still.");

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readMPU(ax, ay, az, gx, gy, gz);

    axSum += ax;
    aySum += ay;
    azSum += az - 16384; // remove gravity if Z is vertical

    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(2);
  }

  axOffset = axSum / (float)CALIBRATION_SAMPLES;
  ayOffset = aySum / (float)CALIBRATION_SAMPLES;
  azOffset = azSum / (float)CALIBRATION_SAMPLES;

  gxOffset = gxSum / (float)CALIBRATION_SAMPLES;
  gyOffset = gySum / (float)CALIBRATION_SAMPLES;
  gzOffset = gzSum / (float)CALIBRATION_SAMPLES;

  Serial.println("# Calibration complete.");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  setupMPU();
  calibrateMPU();

  Serial.println("t_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,a_resultant_g");
}

void loop() {
  static uint32_t lastSample = micros();

  if (micros() - lastSample >= SAMPLE_PERIOD_US) {
    lastSample += SAMPLE_PERIOD_US;

    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    readMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    float ax = (rawAx - axOffset) / ACCEL_SCALE;
    float ay = (rawAy - ayOffset) / ACCEL_SCALE;
    float az = (rawAz - azOffset) / ACCEL_SCALE;

    float gx = (rawGx - gxOffset) / GYRO_SCALE;
    float gy = (rawGy - gyOffset) / GYRO_SCALE;
    float gz = (rawGz - gzOffset) / GYRO_SCALE;

    float aResultant = sqrt(ax * ax + ay * ay + az * az);

    Serial.print(micros());
    Serial.print(",");
    Serial.print(ax, 6);
    Serial.print(",");
    Serial.print(ay, 6);
    Serial.print(",");
    Serial.print(az, 6);
    Serial.print(",");
    Serial.print(gx, 6);
    Serial.print(",");
    Serial.print(gy, 6);
    Serial.print(",");
    Serial.print(gz, 6);
    Serial.print(",");
    Serial.println(aResultant, 6);
  }
}