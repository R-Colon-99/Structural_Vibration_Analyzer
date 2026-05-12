# Structural Vibration Analyzer

Low-cost multi-sensor structural vibration analysis platform using ESP32 and MEMS accelerometers for educational and experimental mechanical vibration studies.

---

# Overview

This project focuses on the development of a portable and affordable vibration-analysis system capable of measuring and analyzing structural vibration using low-cost MEMS accelerometers.

The system is designed primarily for:
- Mechanical vibration experimentation
- Educational laboratory use
- Structural response analysis
- Frequency-domain analysis using FFT
- Comparative sensor studies

The project currently uses multiple MPU6050 accelerometers connected to an ESP32 microcontroller through a PCA9548A I2C multiplexer.

---

# Objectives

- Develop a low-cost vibration analysis platform
- Measure structural vibration at multiple locations simultaneously
- Perform FFT analysis on acceleration data
- Compare structural vibration transmission between locations
- Investigate the limitations of low-cost MEMS sensors
- Create an accessible educational platform for vibration analysis

---

# Current Features

- ESP32-based data acquisition
- Support for 3 simultaneous MPU6050 sensors
- PCA9548A I2C multiplexer integration
- Binary serial data transmission
- Python-based logger and analyzer
- FFT analysis for X, Y, and Z axes
- Comparative FFT overlays between sensors
- Automatic graph generation and export
- Organized raw and processed data workflow

---

# Experimental Setup

Three accelerometers are mounted at different structural locations:

| Sensor | Location |
|---|---|
| S1 | Printer base/frame |
| S2 | Upper structural frame |
| S3 | Print head/extruder |

All sensors are rigidly mounted using screw-fastened brackets to improve measurement consistency and reduce mounting-induced vibration artifacts.

The current primary test platform is an Elegoo Centauri Carbon 3D printer.

---

# Hardware Used

- ESP32 Development Board
- MPU6050 Accelerometers (x3)
- PCA9548A I2C Multiplexer
- Rigid sensor mounting brackets
- USB Serial Communication
- Optional MicroSD module (future integration)

---

# Software Stack

- PlatformIO
- VS Code
- Python
- NumPy
- Matplotlib
- PySerial

---

# Future Work

- SD card standalone logging
- Wireless monitoring interface
- Higher-rate acquisition systems
- ADXL355 sensor integration
- Modal analysis implementation
- Natural frequency estimation tools
- Comparison with industrial accelerometers
- Real-time web dashboard visualization

---

# Author

Raúl Colón  
Mechanical Engineering Student  
Universidad Ana G. Méndez
