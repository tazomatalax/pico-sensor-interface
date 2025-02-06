#include "Arduino.h"
#include "ModbusRTUSlave.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Pins
#define PIN_ADC_0   26
#define PIN_ADC_1   27
#define PIN_ADC_2   28
#define PIN_PULSE_INPUT 22

// 4-2-mA calcs
#define SAMPLES  16
#define ADC_INTERVAL 100
#define mA_PER_BIT 0.00805664062

#define MOTOR_INTERVAL 500

// Lib objects
ModbusRTUSlave modbus(Serial);
SemaphoreHandle_t sensorMutex = NULL;

// Globals
struct sensors_t {
    float ch0_mA = 4.0;
    float ch1_mA = 8.596;
    float ch2_mA = 16.25736;
    float pulse_rate = 0.0;
    float motor_mA = 0.0;
};

sensors_t sensors;
uint16_t holdingRegisters[100];

// ADC pins
uint32_t adc_ch[3] = {PIN_ADC_0, PIN_ADC_1, PIN_ADC_2};

// Timing
uint32_t lastMicros;
uint32_t adcLastMillis;
uint32_t motorLastMillis;