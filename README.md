# Pico Sensor Interface

A Raspberry Pi Pico-based sensor interface project that provides real-time monitoring and control capabilities using Modbus RTU communication protocol. This project is built using PlatformIO and the Arduino framework.

## Features

- Real-time sensor data acquisition using 12-bit ADC
- Pulse rate measurement with interrupt-based detection
- Modbus RTU slave communication (115200 baud)
- Dual-core utilization for efficient task handling
- Motor power control functionality
- Thread-safe sensor data handling using FreeRTOS mutexes

## Hardware Requirements

- Raspberry Pi Pico
- Sensors (compatible with 3.3V logic)
- Motor control interface
- RS-485 interface for Modbus RTU communication

## Software Dependencies

- PlatformIO
- Arduino framework for RP2040 (Earle Philhower core)
- ModbusRTUSlave library

## Getting Started

1. Install PlatformIO IDE (VS Code extension) or PlatformIO Core
2. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/pico-sensor-interface.git
   ```
3. Open the project in PlatformIO
4. Build and upload to your Raspberry Pi Pico

## Project Structure

- `/src` - Main source files
  - `main.cpp` - Primary application logic
  - `sys_init.h` - System initialization and configuration
- `/lib` - Project libraries
  - `ModbusRTUSlave` - Modbus RTU communication library
- `/include` - Header files
- `/test` - Test files

## Configuration

The project can be configured through the `platformio.ini` file. Current settings:

```ini
[env:pico]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = pico
framework = arduino
board_build.core = earlephilhower
```

## Communication Protocol

The device operates as a Modbus RTU slave with the following specifications:
- Slave ID: 1
- Baud Rate: 115200
- Data Format: RTU
- Holding Registers: 10 registers available for control and monitoring

## Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

## License

[Add your chosen license here]

## Author

[Your Name/Organization]
