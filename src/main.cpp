#include "sys_init.h"

// Forward declarations
void pulse_ISR(void);
void handle_ADC(void);
void handle_motor_pwr(void);

// ----------------------------Setup ----------------------------
void setup() {
    analogReadResolution(12);
    // Init power monitoring - INA226?

    // ...
    modbus.configureHoldingRegisters(holdingRegisters, 10);
    modbus.begin(1, 115200);        // Modbus slave ID 1, baud rate 115200
    adcLastMillis = millis();
    lastMicros = micros();
}

void setup1() {
    sensorMutex = xSemaphoreCreateMutex();
    pinMode(PIN_PULSE_INPUT, INPUT_PULLUP);
    attachInterrupt(PIN_PULSE_INPUT, pulse_ISR, FALLING);
}

// ----------------------------Loop ----------------------------
void loop() {
    modbus.poll();
    if (millis() - adcLastMillis > ADC_INTERVAL) handle_ADC();
    if (millis() - motorLastMillis > MOTOR_INTERVAL) handle_motor_pwr();
}

void loop1() {
    delay(1);
}

// ----------------------- ISR callbacks -----------------------

void pulse_ISR(void) {
    uint32_t ts = micros();
    uint32_t period = ts - lastMicros;
    lastMicros = ts;
    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        if (period < 60000000) sensors.pulse_rate = 1000000.0 / period; // Ignore pulse durations longer than 60 seconds
        else sensors.pulse_rate = 0.0;
        xSemaphoreGive(sensorMutex);
    }
}

void handle_ADC(void) {
    adcLastMillis = millis();
    float samples[3];
    // Sample each channel 16 times and average
    for (uint8_t s = 0; s < SAMPLES; s++) {
        for (uint8_t ch = 0; ch < 3; ch++) {
            samples[ch] += analogRead(adc_ch[ch]);
        }
    }
    for (uint8_t ch = 0; ch < 3; ch++) samples[ch] /= SAMPLES;

    // Calculate effective mA measurement for each channel and store to holding registers
    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        sensors.ch0_mA = samples[0] * mA_PER_BIT;
        sensors.ch1_mA = samples[1] * mA_PER_BIT;
        sensors.ch2_mA = samples[2] * mA_PER_BIT;
        memcpy(holdingRegisters, &sensors, sizeof(sensors_t));
        xSemaphoreGive(sensorMutex);
    }
    
}

void handle_motor_pwr(void) {
    motorLastMillis = millis();
    float motor_current = 0.0;
    // Get motor current reading...

    // ...

    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        sensors.motor_mA = motor_current;
        xSemaphoreGive(sensorMutex);
    }
}