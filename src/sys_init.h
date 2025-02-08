#include "Arduino.h"
#include "Wire.h"
#include "ModbusRTUSlave.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Pins
#define PIN_ADC_0       26
#define PIN_ADC_1       27
#define PIN_ADC_2       28
#define PIN_PULSE_INPUT 22
#define PIN_SDA         16
#define PIN_SCL         17

// Serial
#define SerialDebug Serial          // USB serial
#define SerialModbus Serial1        // UART 0

// 4-2-mA calcs
#define SAMPLES  128                // Number of samples per channel measurement (about 100µs per sample)
#define ADC_INTERVAL 100            // Time in milliseconds between ADC measurements
#define ADC_NOISE_FLOOR 40          // ADC LSB noise floor volue (ignore ADC raw values below this number)
#define mA_PER_BIT 0.00805664062    // mA per LSB constant (calculated for 100­Ω shunt resistor, 12-bit ADC, 3.3V reference voltage)

// Motor power update interval (milliseconds)
#define MOTOR_INTERVAL 500

// Stirrer RPM pulse timeout (microseconds)
#define PULSE_TIMEOUT 10000000  // 10 seconds

// Lib objects
ModbusRTUSlave modbus(SerialModbus);
SemaphoreHandle_t sensorMutex = NULL;       // Mutex so both cores can safely access sensor data

// Globals
struct sensors_t {
    float ch0_mA = 4.0;
    float ch1_mA = 8.596;
    float ch2_mA = 16.25736;
    float rpm = 0.0;
    float motor_mA = 0.0;
};

sensors_t sensors;
uint16_t holdingRegisters[100];

// ADC pins
uint32_t adc_ch[3] = {PIN_ADC_0, PIN_ADC_1, PIN_ADC_2};

// Timing
uint32_t rpm_timestamp;
uint32_t rpm_period;
uint32_t rpm_lastMicros;
uint32_t adcLastMillis;
uint32_t motorLastMillis;

// RPM
float rpm = 0.0;
bool rpmUpdated = false;

// Flags
bool debug = true;
bool core0_ready = false, core1_ready = false;