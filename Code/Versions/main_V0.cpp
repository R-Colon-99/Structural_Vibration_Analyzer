#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

bool recording = false;

const int SDA_PIN = 21;
const int SCL_PIN = 22;

const unsigned long SAMPLE_RATE_HZ = 1000;
const unsigned long SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE_HZ;

unsigned long startTimeUs = 0;
unsigned long lastSampleTimeUs = 0;
unsigned long lastToggleTime = 0;

void setup() {
  Serial.begin(921600);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring.");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  Serial.println("Press ENTER to START recording...");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    unsigned long now = millis();

    if ((c == '\n' || c == '\r') && (now - lastToggleTime > 300)) {
      lastToggleTime = now;

      while (Serial.available()) {
        Serial.read();
      }

      recording = !recording;

      if (recording) {
        startTimeUs = micros();
        lastSampleTimeUs = startTimeUs;

        Serial.println("Recording STARTED");
        Serial.println("time_us,ax,ay,az");
      } else {
        Serial.println("Recording STOPPED");
        Serial.println("Press ENTER to start again...");
      }
    }
  }

  if (recording) {
    unsigned long currentTimeUs = micros();

    if (currentTimeUs - lastSampleTimeUs >= SAMPLE_INTERVAL_US) {
      lastSampleTimeUs += SAMPLE_INTERVAL_US;

      sensors_event_t accel, gyro, temp;
      mpu.getEvent(&accel, &gyro, &temp);

      Serial.print(currentTimeUs - startTimeUs);
      Serial.print(",");
      Serial.print(accel.acceleration.x, 4);
      Serial.print(",");
      Serial.print(accel.acceleration.y, 4);
      Serial.print(",");
      Serial.println(accel.acceleration.z, 4);
    }
  }
}