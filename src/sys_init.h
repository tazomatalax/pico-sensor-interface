#include "Arduino.h"
#include "Wire.h"
#include "ModbusRTUSlave.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "INA226.h"

// Pins
#define PIN_ADC_0       26
#define PIN_ADC_1       27
#define PIN_ADC_2       28
#define PIN_PULSE_INPUT 22
#define PIN_SDA         12
#define PIN_SCL         13
#define PIN_UART_TX     16
#define PIN_UART_RX     17


// Serial
#define SerialDebug Serial1          // USB serial
#define SerialModbus Serial        // UART 0

// Modbus
#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUD 115200
#define MODBUS_UNIT_ID 4            // Unit ID in holding register for ident Change this to match the unit ID of the Pico

// 4-2-mA calcs
#define SAMPLES  255                // Number of samples per channel measurement (about 100µs per sample)
#define ADC_INTERVAL 100            // Time in milliseconds between ADC measurements
#define ADC_NOISE_FLOOR 0.5         // ADC noise floor value (mA)

#define mA_PER_BIT 0.00805664062    // mA per LSB constant (calculated for 100­Ω shunt resistor, 12-bit ADC, 3.3V reference voltage)

// Per board calibration adjustment (choose 1) ------------------------------------------>

/*// Pico sensor interface #1
#define UNIT_ID                 "Unit 1"
#define mA_OFFSET_ch0           -0.2            // Channel 0 mA offset
#define mA_OFFSET_ch1           -0.2            // Channel 1 mA offset
#define mA_OFFSET_ch2           -0.2            // Channel 2 mA offset
#define mA_OFFSET_ina226        0.0             // Motor current sensor mA offset
#define mA_MULTIPLIER_ch0       0.991           // Channel 0 mA multiplier
#define mA_MULTIPLIER_ch1       0.991           // Channel 1 mA multiplier
#define mA_MULTIPLIER_ch2       0.991           // Channel 2 mA multiplier
#define mA_MULTIPLIER_ina226    0.916           // Motor current sensor mA multiplier*/

/*// Pico sensor interface #2 
#define UNIT_ID                 "Unit 2"
#define mA_OFFSET_ch0           -0.08           // Channel 0 mA offset
#define mA_OFFSET_ch1           -0.08           // Channel 1 mA offset
#define mA_OFFSET_ch2           -0.08           // Channel 2 mA offset
#define mA_OFFSET_ina226        0.0             // Motor current sensor mA offset
#define mA_MULTIPLIER_ch0       0.99            // Channel 0 mA multiplier
#define mA_MULTIPLIER_ch1       0.99            // Channel 1 mA multiplier
#define mA_MULTIPLIER_ch2       0.99            // Channel 2 mA multiplier
#define mA_MULTIPLIER_ina226    0.91            // Motor current sensor mA multiplier*/

/*// Pico sensor interface #3 
#define UNIT_ID                 "Unit 3"
#define mA_OFFSET_ch0           -0.17           // Channel 0 mA offset
#define mA_OFFSET_ch1           -0.17           // Channel 1 mA offset
#define mA_OFFSET_ch2           -0.17           // Channel 2 mA offset
#define mA_OFFSET_ina226        2.0             // Motor current sensor mA offset
#define mA_MULTIPLIER_ch0       0.991           // Channel 0 mA multiplier
#define mA_MULTIPLIER_ch1       0.991           // Channel 1 mA multiplier
#define mA_MULTIPLIER_ch2       0.991           // Channel 2 mA multiplier
#define mA_MULTIPLIER_ina226    0.921           // Motor current sensor mA*/

// Pico sensor interface #4 
#define UNIT_ID                 "Unit 4"
#define mA_OFFSET_ch0           -0.22           // Channel 0 mA offset
#define mA_OFFSET_ch1           -0.22           // Channel 1 mA offset
#define mA_OFFSET_ch2           -0.22           // Channel 2 mA offset
#define mA_OFFSET_ina226        -3.0            // Motor current sensor mA offset
#define mA_MULTIPLIER_ch0       0.991           // Channel 0 mA multiplier
#define mA_MULTIPLIER_ch1       0.991           // Channel 1 mA multiplier
#define mA_MULTIPLIER_ch2       0.991           // Channel 2 mA multiplier
#define mA_MULTIPLIER_ina226    0.897           // Motor current sensor mA multiplier

// <--------------------------------------------------------------------------------------

// Motor power update interval (milliseconds)
#define MOTOR_INTERVAL 500

// Stirrer RPM pulse timeout (microseconds)
#define PULSE_TIMEOUT 20000000  // 20 seconds

// Minimum pulse rpm_period
#define MIN_RPM_PERIOD 1000000 // 1 second

// ISR wait time (checks for brief pulse and ignores it)
#define ISR_WAIT_TIME 10000   // 10 milliseconds

// Status LED on/off interval (milliseconds)
#define LED_INTERVAL 500

// Lib objects
ModbusRTUSlave modbus(SerialModbus);
SemaphoreHandle_t sensorMutex = NULL;       // Mutex so both cores can safely access sensor data
INA226 ina226(0x40);                        // INA226 sensor object

// Globals
struct sensors_t {
    float ch0_mA;
    float ch1_mA;
    float ch2_mA;
    float rpm;
    float motor_mA;
    uint16_t unit_id;
};

sensors_t sensors;
uint16_t holdingRegisters[100];

// ADC pins and calibration
uint32_t adc_ch[3] = {PIN_ADC_0, PIN_ADC_1, PIN_ADC_2};
float adc_multiplier[3] = {mA_MULTIPLIER_ch0, mA_MULTIPLIER_ch1, mA_MULTIPLIER_ch2};
float adc_offset[3] = {mA_OFFSET_ch0, mA_OFFSET_ch1, mA_OFFSET_ch2};

// Timing
uint32_t rpm_timestamp;
uint32_t rpm_period;
uint32_t rpm_lastMicros;
uint32_t adcLastMillis;
uint32_t motorLastMillis;
uint32_t ledLastMillis;

// RPM
float rpm = 0.0;
bool rpmUpdated = false;

// Flags
bool debug = false;
bool core0_ready = false, core1_ready = false;