#include "sys_init.h"

// Forward declarations
void pulse_ISR(void);
void handle_ADC(void);
void handle_motor_pwr(void);
bool set_rpm(float rpm_val);
bool check_pulse_timeout(void);

// ----------------------------Setup ----------------------------
void setup() {
    if (debug) {
        SerialDebug.begin(115200);
        while (!SerialDebug);
        SerialDebug.println("Starting pico sensor interface in debug mode.");
    }
    analogReadResolution(12);
    // TODO: Init power monitoring - INA226?
    /*Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);*/
    // ...
    
    // DEBUG - Setup modbus serial port on UART0 (debug mode only - frees up USB for uploads and debugging)
    Serial1.setTX(PIN_SDA);
    Serial1.setRX(PIN_SCL);
    // END DEBUG

    modbus.configureHoldingRegisters(holdingRegisters, 10);
    modbus.begin(1, 115200);        // Modbus slave ID 1, baud rate 115200
    adcLastMillis = millis();
    rpm_lastMicros = micros();
    
    core0_ready = true;
    while (!core1_ready);
    if (debug) SerialDebug.println("Setup complete.");

    if (debug) SerialDebug.printf("A0: %u, A1: %u, A2: %u\n", analogRead(PIN_ADC_0), analogRead(PIN_ADC_1), analogRead(PIN_ADC_2));
}

void setup1() {
    sensorMutex = xSemaphoreCreateMutex();
    pinMode(PIN_PULSE_INPUT, INPUT_PULLUP);
    attachInterrupt(PIN_PULSE_INPUT, pulse_ISR, FALLING);
    if (debug) SerialDebug.println("Core 1 setup complete.");
    core1_ready = true;
    while (!core0_ready);
}

// ----------------------------Loop ----------------------------
void loop() {
    modbus.poll();
    if (millis() - adcLastMillis > ADC_INTERVAL) handle_ADC();
    if (millis() - motorLastMillis > MOTOR_INTERVAL) handle_motor_pwr();
}

void loop1() {
    delay(300);
    if (rpmUpdated) set_rpm(rpm);
    else (check_pulse_timeout());
}

// ----------------------- ISR callbacks -----------------------

void pulse_ISR(void) {
    rpm_timestamp = micros();
    rpm_period = rpm_timestamp - rpm_lastMicros;
    rpm_lastMicros = rpm_timestamp;
    if (rpm_period < 60000000) rpm = (1000000.0 / rpm_period) * 60;
    else rpm = 0.0;  // Ignore pulse durations longer than 60 seconds
    rpmUpdated = true;
}

void handle_ADC(void) {
    adcLastMillis = millis();
    uint32_t samples[3] = {0, 0, 0};
    // Sample each channel 16 times and average
    for (uint8_t s = 0; s < SAMPLES; s++) {
        for (uint8_t ch = 0; ch < 3; ch++) {
            samples[ch] += analogRead(adc_ch[ch]);
        }
    }
    //if (debug) SerialDebug.printf("16x oversampled - A0: %u, A1: %u, A2: %u\n", samples[0], samples[1], samples[2]);
    for (uint8_t ch = 0; ch < 3; ch++) {
        samples[ch] /= SAMPLES;
        if (samples[ch] < 20) samples[ch] = 0.0; // Ignore noise
    }

    // Calculate effective mA measurement for each channel and store to holding registers
    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        sensors.ch0_mA = (float)samples[0] * mA_PER_BIT;
        sensors.ch1_mA = (float)samples[1] * mA_PER_BIT;
        sensors.ch2_mA = (float)samples[2] * mA_PER_BIT;
        memcpy(holdingRegisters, &sensors, sizeof(sensors_t));
        xSemaphoreGive(sensorMutex);
    }
    if (debug) {
        uint32_t adc_time = millis() - adcLastMillis;
        SerialDebug.printf("ADC oversampled %ix, time: %ums\n", SAMPLES, adc_time);
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

bool set_rpm(float rpm_val) {
    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        sensors.rpm = rpm_val;
        xSemaphoreGive(sensorMutex);
        rpmUpdated = false;
        return true;
    }
    return false;
}

bool check_pulse_timeout(void) {
    if (micros() - rpm_lastMicros < PULSE_TIMEOUT) return false;
    rpm = 0.0;
    set_rpm(0.0);
    return true;
}