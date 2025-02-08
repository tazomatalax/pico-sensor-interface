# Pico Sensor Interface

NOTE: This code is WIP and in very early dev.

A Raspberry Pi Pico-based sensor interface project that provides real-time monitoring and data transmission using Modbus RTU communication protocol. This project is built using PlatformIO and the Arduino framework.

## Features

- Real-time sensor data acquisition using 12-bit ADC
- Pulse rate measurement with interrupt-based detection
- Modbus RTU slave communication (115200 baud)
- Dual-core utilization for efficient task handling
- Motor power control functionality
- Thread-safe sensor data handling using FreeRTOS mutexes

## Hardware Requirements

- Raspberry Pi Pico
- 4 - 20mA Sensors (100R shunt resistor)
- Active low RPM sensor (1 pulse per revolution)
- Current monitoring over I2C
- USB Serial Modbus RTU communication

![Basic circuit diagram](https://github.com/ScionResearch/pico-sensor-interface/tree/main/schematic/circuit.png)

## Software Dependencies

- PlatformIO
- Arduino framework for RP2040 (Earle Philhower core)
- ModbusRTUSlave library

## Getting Started

1. Install PlatformIO IDE (VS Code extension) or PlatformIO Core
2. Clone this repository:
   ```bash
   git clone https://github.com/ScionResearch/pico-sensor-interface.git
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

## Modbus Register Map

The following table describes the Modbus holding registers used in this device:

| Register Address | Name | Data Type | Description |
|-----------------|------|------------|-------------|
| 40001 (0x0000) | CH0_MA | FLOAT32 | Channel 0 current measurement in mA |
| 40003 (0x0002) | CH1_MA | FLOAT32 | Channel 1 current measurement in mA |
| 40005 (0x0004) | CH2_MA | FLOAT32 | Channel 2 current measurement in mA |
| 40007 (0x0006) | PULSE_RATE | FLOAT32 | Measured pulse rate in Hz |
| 40009 (0x0008) | MOTOR_MA | FLOAT32 | Motor current measurement in mA |

Notes:
- All FLOAT32 values use two consecutive registers (32 bits)
- Register addresses follow the standard Modbus convention where 4xxxx indicates holding registers
- The device uses standard IEEE 754 floating-point format
- Values are updated every ADC_INTERVAL for current measurements and in real-time for pulse rate

## Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

## License

[Add your chosen license here]

## Author

[Your Name/Organization]
