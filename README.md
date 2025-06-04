# AstriCore - Real-Time Hardware Control & Data Acquisition
=======

## Overview
AstriCore is a multi-threaded firmware platform for Teensy 4.1 microcontrollers, designed for real-time hardware control and sensor data acquisition. It provides a robust foundation for precision mechatronic systems requiring coordinated motor control, load sensing, and position feedback.

## Features
- **Multi-threaded Architecture**: Concurrent control, sensor, and communication tasks using TeensyThreads
- **Motor Control**: Pulse-pair stepper motor driving with acceleration profiles and velocity control
- **Sensor Integration**: Load cell and rotary encoder data acquisition with thread-safe access
- **JSON Communication**: Structured serial protocol for real-time command and telemetry exchange
- **Hardware Abstraction**: Clean interfaces for motors, sensors, and communication peripherals

## Core Capabilities
### Real-Time Control
- Stepper motor velocity control with configurable acceleration/deceleration profiles
- Thread-safe setpoint management and hardware interfacing
- Microsecond-precision pulse generation with hardware timers

### Data Acquisition
- Load cell reading via HX711 interface
- Quadrature encoder position tracking
- High-frequency sensor sampling with mutex-protected data sharing

### Communication
- USB serial interface with JSON message protocol
- Bidirectional command and telemetry exchange
- Real-time status reporting and error handling

## Hardware Platform
- **Target**: Teensy 4.1 microcontroller
- **Development**: PlatformIO with Arduino framework
- **Interfaces**: Stepper motor drivers, load cells, rotary encoders
- **Communication**: USB serial for host integration

## Project Structure
```
AstriCore/
├── Teensy4.1/           # PlatformIO project
│   ├── src/             # Source code
│   │   ├── main.cpp     # Multi-threaded main application
│   │   └── PulsePairSteppers.*  # Motor control library
│   └── include/         # Hardware definitions and pinouts
```

## Development Setup
1. Install PlatformIO with Teensy 4.1 support
2. Clone repository and open Teensy4.1 project folder
3. Build and upload firmware via PlatformIO

## Integration
AstriCore serves as the embedded foundation for higher-level control systems and can be integrated with:
- Host applications for GUI control and data visualization
- Automated test sequences and data logging systems
- Calibration and configuration management tools

## Architecture Philosophy
The firmware emphasizes real-time responsiveness, thread safety, and clean separation between control logic, sensor management, and communication protocols. This modular approach enables reliable operation in demanding mechatronic applications while maintaining extensibility for future hardware integration.

## License
Licensed under Apache License 2.0 for software components. See LICENSE file for details.

## Contact
For questions or collaboration, contact clayton@astrisdesign.com