# Changelog

All notable changes and development progress for the Structural Vibration Analyzer project are documented here.

---

# [Initial Concept Phase]

## Project Planning
### Added
- Initial concept for a low-cost structural vibration analyzer
- Goal of developing an educational vibration-analysis platform
- Preliminary research into vibration-analysis methods and FFT processing
- Evaluation of low-cost MEMS accelerometers for vibration measurements

### Research
- Investigated industrial vibration-analysis systems
- Compared commercial accelerometers and vibration analyzers
- Studied vibration-analysis applications in industry
- Evaluated feasibility for educational laboratory implementation

---

# [Arduino Mega Development Phase]

## Initial Hardware Setup
### Added
- Arduino Mega 2560 initial acquisition platform
- Single MPU6050 accelerometer integration
- Basic acceleration acquisition over I2C
- Serial acceleration output to PC

### Implemented
- Time-domain acceleration measurements
- Initial FFT processing
- Basic graph visualization

### Tested
- Impact testing on structural systems
- Preliminary vibration measurements on mechanical structures

---

# [Higher Sampling Rate Development]

## Sampling Optimization
### Improved
- Increased sampling frequency on Arduino Mega
- Reduced serial bottlenecks
- Improved timing consistency

### Investigated
- Maximum achievable MPU6050 sampling rate
- Limitations of Arduino Mega serial communication
- Data-transfer bottlenecks during high-frequency acquisition

### Result
- Determined need for higher-performance microcontroller platform

---

# [ESP32 Migration Phase]

## ESP32 Integration
### Added
- ESP32-based acquisition system
- Improved processing capability
- Higher-speed serial communication
- Support for higher sampling frequencies

### Improved
- Sampling stability
- Real-time acquisition performance
- Data throughput capabilities

### Fixed
- Serial communication timing issues
- Logger synchronization problems

---

# [Multi-Sensor Expansion]

## Three-Sensor System
### Added
- Support for three MPU6050 accelerometers
- PCA9548A I2C multiplexer integration
- Simultaneous multi-sensor acquisition

### Implemented
- Sensor channel switching
- Independent sensor reading system
- Multi-location structural vibration measurements

### Sensor Locations
- S1 mounted at printer base/frame
- S2 mounted at upper frame structure
- S3 mounted at print head/extruder assembly

---

# [Binary Communication Implementation]

## Data Transmission Optimization
### Added
- Binary serial data transmission
- Compact packet structure
- Faster PC-side data reception

### Improved
- Higher effective sampling rates
- Reduced serial overhead
- More stable high-frequency acquisition

### Result
- Stable operation achieved near 400–800 Hz sampling rates

---

# [Python Logger Development]

## Logging System
### Added
- Python-based serial logger
- Automatic file creation
- Timestamped recordings
- Structured raw-data storage

### Improved
- User interaction during acquisition
- Dataset management workflow
- File organization system

### Fixed
- Logger startup synchronization
- Serial buffer issues
- Data corruption during long recordings

---

# [FFT Analyzer Development]

## Signal Processing Expansion
### Added
- FFT processing for X, Y, and Z axes independently
- Resultant acceleration analysis
- Automatic FFT graph generation
- Processed data export system

### Implemented
- Frequency-domain visualization
- Axis-by-axis vibration analysis
- Comparative FFT analysis between sensors

### Added Graph Features
- Frequency tick marks
- Adjustable grid spacing
- Automatic graph saving
- Organized processed-data folders

---

# [Experimental Testing Phase]

## Structural Vibration Experiments
### Tested
- Impact-response measurements
- Operational vibration analysis
- Multi-location structural response

### Observed
- Dominant low-frequency structural behavior
- Vibration transmission differences across printer structure
- Higher vibration amplitudes near moving print head assembly

### Preliminary Findings
- Estimated structural resonance behavior near approximately 6 Hz
- Increased excitation magnitude improved frequency visibility
- Rigid sensor mounting significantly improved data quality

---

### Research Goals
- Evaluate limitations of low-cost sensors
- Determine educational usefulness
- Develop accessible vibration-analysis tools for students

---

# [Current State]

## Current Capabilities
### System Features
- ESP32-based acquisition
- Three simultaneous accelerometers
- PCA9548A multiplexer support
- Binary serial communication
- High-rate sampling
- Python logger and analyzer
- FFT analysis for all axes
- Multi-sensor FFT overlays
- Automatic graph export system

### Experimental Platform
- Elegoo Centauri Carbon 3D printer
- Rigid multi-sensor mounting system
- Operational and impact testing capability

### Current Focus
- Improving measurement quality
- Increasing acquisition performance
- Developing educational vibration-analysis workflow
- Expanding experimental validation

---

# [Planned Future Development]

## Future Work
### Planned
- SD card standalone logging
- Wireless monitoring system
- Real-time web dashboard
- Higher-end accelerometer integration
- ADXL355 implementation
- Teensy 4.1 or STM32 migration
- Modal analysis tools
- Natural frequency estimation automation
- Industrial sensor comparison studies
- Calibration and validation procedures
