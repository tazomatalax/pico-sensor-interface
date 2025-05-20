#include "sys_init.h"
// #include <Wire.h>
//
// // AS5600 I2C address and register
// #define AS5600_ADDR 0x36
// #define AS5600_ANGLE_HI 0x0E
// #define AS5600_ANGLE_LO 0x0F
//
// // AS5600 RPM calculation state
// volatile uint16_t as5600_prev_angle = 0;
// volatile uint32_t as5600_prev_time = 0;
// volatile float as5600_rpm = 0.0;
// #define AS5600_SAMPLE_INTERVAL 50 // ms
// #define AS5600_MA_FILTER_SIZE 5
// float as5600_rpm_history[AS5600_MA_FILTER_SIZE] = {0};
// uint8_t as5600_rpm_idx = 0;
//
// // Forward declarations
void pulse_ISR(void);
void handle_ADC(void);
// void handle_current_sensor(void);
void handle_led(void);
bool set_rpm(float rpm_val);
bool check_pulse_timeout(void);
// uint16_t as5600_read_angle();
// void handle_as5600_rpm(void);

// ----------------------------Setup ----------------------------

// Core 0 - General purpose with basic timing requirements
void setup() {
    if (debug) {
        SerialDebug.begin(115200);
        while (!SerialDebug);
        SerialDebug.printf("Starting %s pico sensor interface in debug mode.\n", UNIT_ID);
    }
    analogReadResolution(12);
    pinMode(LED_BUILTIN, OUTPUT);

//     Wire.setSDA(PIN_SDA);
//     Wire.setSCL(PIN_SCL);
//     Wire.begin();
//     if (!ina226.begin()) {
//         if (debug) SerialDebug.println("ERROR: Failed to start INA226 current sensor interface");    
//         while (1);
//     }
//     if (debug) SerialDebug.println("INA226 current sensor interface started");
//     ina226.setAverage(INA226_1024_SAMPLES);
//     uint16_t err = ina226.setMaxCurrentShunt(8, 0.01, true);
//     char *shunt_err[4] = {"shunt voltage high", "max current low", "shunt low", "normalize fail"};
//     if (debug && err != 0) SerialDebug.printf("ERROR: Failed to set INA226 shunt current limit, reason: %s\r\n", shunt_err[err & 3]);
    
    // DEBUG - Setup modbus serial port on UART0 (debug mode only - frees up USB for uploads and debugging)
    Serial1.setTX(PIN_UART_TX);
    Serial1.setRX(PIN_UART_RX);
    // END DEBUG
    sensors.unit_id = MODBUS_UNIT_ID;  // Set unit ID in struct
    modbus.configureHoldingRegisters(holdingRegisters, 100);
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
    attachInterrupt(PIN_PULSE_INPUT, pulse_ISR, RISING);
    if (debug) SerialDebug.println("Core 1 setup complete.");
    core1_ready = true;
    while (!core0_ready);
}

// ----------------------------Loop ----------------------------
void loop() {
    modbus.poll();
    if (millis() - adcLastMillis > ADC_INTERVAL) handle_ADC();
    // if (millis() - motorLastMillis > MOTOR_INTERVAL) handle_current_sensor();
    if (millis() - ledLastMillis > LED_INTERVAL) handle_led();
    // if (millis() - as5600_prev_time > AS5600_SAMPLE_INTERVAL) handle_as5600_rpm();
}

void loop1() {
    delay(300);
    if (rpmUpdated) set_rpm(rpm);
    else (check_pulse_timeout());
}

// ----------------------- ISR callbacks -----------------------

void pulse_ISR(void) {
    delayMicroseconds(ISR_WAIT_TIME);  // Wait for 10ms to avoid false triggers
    if (digitalRead(PIN_PULSE_INPUT) == LOW) return;  // Ignore if the pin is low
    rpm_timestamp = micros();
    float temp_period = rpm_timestamp - rpm_lastMicros;
    if (temp_period < MIN_RPM_PERIOD) return;  // Ignore very short pulses
    rpm_period = temp_period;
    rpm_lastMicros = rpm_timestamp;
    if (rpm_period < 60 * PULSE_TIMEOUT) rpm = (1000000.0 / rpm_period) * 60;
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
    /*if (debug) {
        uint32_t adc_time = millis() - adcLastMillis;
        SerialDebug.printf("ADC oversampled %ix, time: %ums\n", SAMPLES, adc_time);
    }*/

    if (debug) {
        SerialDebug.printf("ADC ch0: %fmA, ch1: %fmA, ch2: %fmA\n", calculated_values[0], calculated_values[1], calculated_values[2]);
    }
}

// Update motor current from INA226? sensor
// void handle_current_sensor(void) {
//     motorLastMillis += MOTOR_INTERVAL;
//     float motor_current = ina226.getCurrent_mA();
//     motor_current = mA_OFFSET_ina226 + motor_current * mA_MULTIPLIER_ina226;
//     if (debug) {
//         SerialDebug.printf("%fV, %fV, %fmA, %fW\n", ina226.getBusVoltage(), ina226.getShuntVoltage(), motor_current, ina226.getPower());
//     }
//
//     if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
//         sensors.motor_mA = motor_current;
//         xSemaphoreGive(sensorMutex);
//     }
// }

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

// // Read 12-bit angle from AS5600
// uint16_t as5600_read_angle() {
//     Wire.beginTransmission(AS5600_ADDR);
//     Wire.write(AS5600_ANGLE_HI);
//     Wire.endTransmission(false);
//     Wire.requestFrom(AS5600_ADDR, 2u);
//     uint8_t hi = Wire.read();
//     uint8_t lo = Wire.read();
//     return ((hi << 8) | lo) & 0x0FFF;
// }
// 
// // Handle AS5600 RPM calculation
// void handle_as5600_rpm() {
//     static bool initialized = false;
//     uint32_t now = millis();
//     uint16_t angle = as5600_read_angle();
//     if (!initialized) {
//         as5600_prev_angle = angle;
//         as5600_prev_time = now;
//         initialized = true;
//         return;
//     }
//     int16_t delta = angle - as5600_prev_angle;
//     if (delta > 2048) delta -= 4096;
//     if (delta < -2048) delta += 4096;
//     uint32_t dt = now - as5600_prev_time;
//     if (dt > 0) {
//         float rpm = fabsf((float)delta * 60.0f / 4096.0f / ((float)dt / 1000.0f));
//         as5600_rpm_history[as5600_rpm_idx] = rpm;
//         as5600_rpm_idx = (as5600_rpm_idx + 1) % AS5600_MA_FILTER_SIZE;
//         float sum = 0.0f;
//         for (uint8_t i = 0; i < AS5600_MA_FILTER_SIZE; i++) sum += as5600_rpm_history[i];
//         as5600_rpm = sum / AS5600_MA_FILTER_SIZE;
//         if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
//             sensors.rpm = as5600_rpm;
//             xSemaphoreGive(sensorMutex);
//         }
//         if (debug) SerialDebug.printf("AS5600: raw delta=%d, dt=%lu ms, rpm=%.2f\n", delta, dt, as5600_rpm);
//     }
//     as5600_prev_angle = angle;
//     as5600_prev_time = now;
// }