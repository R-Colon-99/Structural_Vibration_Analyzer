# Structural_Vibration_Analyzer
Low-cost structural vibration analyzer using ESP32/Arduino and MEMS accelerometers for FFT-based vibration analysis and experimental validation.


## Fusion 360
The CAD model is designed for use with one M3*16 bolt to mount to desired surface, and a small screw to hold sensor firmly in place.

## Project Update Log — Structural Vibration Analyzer
### May 2026 — ESP32 Migration and Multi-Sensor Integration
    Migrated the vibration acquisition system from Arduino Mega to ESP32 to support higher sampling rates and improved serial throughput.
    Implemented support for three MPU6050 accelerometers simultaneously using a PCA9548A I2C multiplexer.
    Verified successful communication between ESP32, PCA9548A, and multiple MPU6050 sensors.
    Added binary serial transmission mode to reduce communication overhead and improve real-time data streaming performance.
    Increased achievable sampling rates from initial low-frequency tests to stable operation near 400–800 Hz.
    Began evaluating the practical upper sampling-rate limits of MPU6050-based low-cost systems.