#include "sys_init.h"

// Forward declarations
void pulse_ISR(void);
void handle_ADC(void);
void handle_current_sensor(void);
void handle_led(void);
bool set_rpm(float rpm_val);
bool check_pulse_timeout(void);

// ----------------------------Setup ----------------------------

// Core 0 - General purpose with basic timing requirements
void setup() {
    if (debug) {
        SerialDebug.begin(115200);
        while (!SerialDebug);
        SerialDebug.println("Starting pico sensor interface in debug mode.");
    }
    analogReadResolution(12);
    pinMode(LED_BUILTIN, OUTPUT);

    // TODO: Init power monitoring - INA226?
    /*Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);*/
    // ...
    
    // DEBUG - Setup modbus serial port on UART0 (debug mode only - frees up USB for uploads and debugging)
    Serial1.setTX(PIN_SDA);
    Serial1.setRX(PIN_SCL);
    // END DEBUG

    modbus.configureHoldingRegisters(holdingRegisters, 10);
    modbus.begin(MODBUS_SLAVE_ID, MODBUS_BAUD);        // Modbus slave ID 1, baud rate 115200
    adcLastMillis = millis();
    rpm_lastMicros = micros();
    ledLastMillis = millis();
    
    core0_ready = true;
    while (!core1_ready);
    if (debug) SerialDebug.println("Setup complete.");
    digitalWrite(LED_BUILTIN, HIGH);
}

// Core 1 - RPM measurement with low overhead for accurate timing measurment
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
    if (millis() - motorLastMillis > MOTOR_INTERVAL) handle_current_sensor();
    if (millis() - ledLastMillis > LED_INTERVAL) handle_led();
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

// ----------------------- Utility functions -------------------

void handle_ADC(void) {
    adcLastMillis += ADC_INTERVAL;
    uint32_t samples[3] = {0, 0, 0};
    float calculated_values[3] = {0.0, 0.0, 0.0};

    // Sample each channel for SAMPLES number of times
    for (uint32_t s = 0; s < SAMPLES; s++) {
        for (uint8_t ch = 0; ch < 3; ch++) {
            samples[ch] += analogRead(adc_ch[ch]);
        }
    }

    // Calculate effective mA measurement for each channel
    for (uint8_t ch = 0; ch < 3; ch++) {
        calculated_values[ch] = ((float)samples[ch] / SAMPLES) * mA_PER_BIT * adc_multiplier[ch] + adc_offset[ch];
        if (calculated_values[ch] < ADC_NOISE_FLOOR) calculated_values[ch] = 0.0; // Assume 0 if we're in the noise floor
    }

    // Store calculated values to sensor data registers
    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        sensors.ch0_mA = calculated_values[0];
        sensors.ch1_mA = calculated_values[1];
        sensors.ch2_mA = calculated_values[2];
        // Copy all sensor data to holding registers (done once here as this is a consistent and fast loop)
        memcpy(holdingRegisters, &sensors, sizeof(sensors_t));
        xSemaphoreGive(sensorMutex);
    }

    // Used to check ADC reads are not taking too long due to SAMPLES setting
    if (debug) {
        uint32_t adc_time = millis() - adcLastMillis;
        SerialDebug.printf("ADC oversampled %ix, time: %ums\n", SAMPLES, adc_time);
    }
}

// Update motor current from INA226? sensor
void handle_current_sensor(void) {
    motorLastMillis += MOTOR_INTERVAL;
    float motor_current = 0.0;
    // Get motor current reading...

    // ...

    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        sensors.motor_mA = motor_current;
        xSemaphoreGive(sensorMutex);
    }
}

// Toggle status LED
void handle_led(void) {
    ledLastMillis += LED_INTERVAL;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// Sets the input RPM value safely to the sensors struct
bool set_rpm(float rpm_val) {
    if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
        sensors.rpm = rpm_val;
        rpmUpdated = false;
        xSemaphoreGive(sensorMutex);
        return true;
    }
    return false;
}

// Checks for a long pulse interval - indicating RPM is probably 0
bool check_pulse_timeout(void) {
    if (micros() - rpm_lastMicros < PULSE_TIMEOUT) return false;
    rpm = 0.0;
    set_rpm(0.0);
    return true;
}