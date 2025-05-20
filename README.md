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
- **Current sensing code is present but disabled by default for hardware compatibility**

## Hardware Requirements

- Raspberry Pi Pico
- 4 - 20mA Sensors (100R shunt resistor)
- Active low RPM sensor (1 pulse per revolution)
- Current monitoring over I2C (INA226, optional/disabled by default)
- USB Serial Modbus RTU communication

![Basic circuit diagram](/schematic/circuit.png)

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

## Important Note on Current Sensing (INA226)

The codebase includes support for current sensing using the INA226 over I2C. However, due to a hardware issue, all INA226-related code (initialization, runtime, and configuration) is commented out by default in `main.cpp` and `sys_init.h`.

- **To enable current sensing:**
  - Uncomment the INA226-related lines in `main.cpp` and `sys_init.h`.
  - Ensure your hardware is working and the INA226 is connected properly.
- **If left commented:**
  - The rest of the system (ADC, RPM, Modbus, etc.) will function normally.
  - No current measurements will be reported in the Modbus registers.

This approach allows easy re-enabling of current sensing in the future by simply uncommenting the relevant code blocks.

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
| 40009 (0x0008) | MOTOR_MA | FLOAT32 | Motor current measurement in mA (only if INA226 enabled) |

Notes:
- All FLOAT32 values use two consecutive registers (32 bits)
- Register addresses follow the standard Modbus convention where 4xxxx indicates holding registers
- The device uses standard IEEE 754 floating-point format
- Values are updated every ADC_INTERVAL for current measurements and in real-time for pulse rate

## Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

## License

Not licensed

## Author

Jeremy Peake - Scion Research
