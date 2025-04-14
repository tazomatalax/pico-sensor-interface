// --- Configuration Section (Easily Editable) ---

const deviceIdPrefix = "pico_";  // Prefix for device IDs

// Calibration/Scaling factors.  These are now *PER DEVICE* and *PER SENSOR*.
const scalingFactors = {
    "pico_1": {
        Temperature: { sensorType: "mA_INPUT", rangeMin: -10, rangeMax: 150 }, // Example: 0-100 degrees
        pH:          { sensorType: "mA_INPUT", rangeMin: -1, rangeMax: 15  }, // Example: 0-14 pH
        ORP:         { sensorType: "mA_INPUT", rangeMin: -1000, rangeMax: +0 },// Example: -500 to +500 mV
        motor_mA:    { multiplier: 0.1, offset: 0.0, sensorType: "INA226" },
        RPM:         { multiplier: 1, offset: 0, sensorType: "PULSE" }
    },
    "pico_2": {
        Temperature: { sensorType: "mA_INPUT", rangeMin: -10, rangeMax: 150 },
        pH:          { sensorType: "mA_INPUT", rangeMin: -1, rangeMax: 15 },
        ORP:         { sensorType: "mA_INPUT", rangeMin: -1000, rangeMax: +0 },
        motor_mA:    { multiplier: 0.1, offset: 0.0, sensorType: "INA226" },
        RPM:         { multiplier: 1, offset: 0, sensorType: "PULSE" }
    },
    "pico_3": {
        Temperature: { sensorType: "mA_INPUT", rangeMin: 0, rangeMax: 100 },
        pH:          { sensorType: "mA_INPUT", rangeMin: 0, rangeMax: 14 },
        ORP:         { sensorType: "mA_INPUT", rangeMin: -1000, rangeMax: +0 },
        motor_mA:    { multiplier: 0.1, offset: 0.0, sensorType: "INA226" },
        RPM:         { multiplier: 1, offset: 0, sensorType: "PULSE" }
    },
    "pico_4": {
        Temperature: { sensorType: "mA_INPUT", rangeMin: 0, rangeMax: 100 },
        pH:          { sensorType: "mA_INPUT", rangeMin: 0, rangeMax: 14 },
        ORP:         { sensorType: "mA_INPUT", rangeMin: -1000, rangeMax: +0 },
        motor_mA:    { multiplier: 0.1, offset: 0.0, sensorType: "INA226" },
        RPM:         { multiplier: 1, offset: 0, sensorType: "PULSE" }
    },
};

// Define valid ranges for each measurement, *after* final scaling.  These are your REAL-WORLD units.
const validRanges = {
    Temperature: { min: -1, max: 110 },  // Allow slight negative/overrange
    pH:          { min: -0.5, max: 14.5 },
    ORP:         { min: -2100, max: 2100 },
    motor_mA:    { min: -5, max: 2000 },
    RPM:         { min: 0, max: 250 }  // Slightly above expected max, for margin.
};

// Noise floor for 4-20mA signals.
const noiseFloor_mA = 0.5;

// Measurement name for InfluxDB
const influxMeasurement = 'pico_measurements';

// --- Additional Configuration for RPM Handling ---
const maxReasonableRPM = 250; // Maximum RPM, slightly above expected max.

// --- End Configuration Section ---

// --- Helper Functions ---

function scale_mA_Input(value, rangeMin, rangeMax) {
    // 1. Apply noise floor (in mA).
    if (value < noiseFloor_mA) {
        value = 0;
    }

    // 2. Convert 4-20mA to 0-100%.
    const percent = (value - 4) / 16;

    // 3. Clamp the percentage.
    const clampedPercent = Math.max(0, Math.min(1, percent));

    // 4. Scale to engineering units.
    const scaledValue = rangeMin + clampedPercent * (rangeMax - rangeMin);
    return scaledValue;
}


function validateAndScale(value, fieldName, deviceId) {
    if (typeof value !== 'number' || isNaN(value) || !isFinite(value)) {
        node.warn(`Invalid ${fieldName} for ${deviceId}: ${value} (Not a valid number)`);
        return null;
    }

    const deviceScaling = scalingFactors[deviceId] || {};
    const scaling = deviceScaling[fieldName] || { sensorType: "UNKNOWN", rangeMin: 0, rangeMax: 1 };

    let scaledValue = value;

    // Apply scaling based on sensor type
    if (scaling.sensorType === "mA_INPUT") {
        // Scale the mA value to engineering units.
        scaledValue = scale_mA_Input(scaledValue, scaling.rangeMin, scaling.rangeMax);

    } else if (scaling.sensorType === "INA226") {
        // Apply offset and multiplier (for motor current).
        scaledValue = value + scaling.offset;
        scaledValue = scaledValue * scaling.multiplier;
    } else if (scaling.sensorType === "PULSE") {
        // RPM-Specific Handling:
        if (scaledValue > maxReasonableRPM) {
             node.warn(`Invalid ${fieldName} for ${deviceId}: ${scaledValue} (Exceeds max reasonable RPM)`);
             return null; // Reject unreasonably large RPM values
        }
    }

    // Range validation *after* scaling
    const range = validRanges[fieldName];
    if (range && (scaledValue < range.min || scaledValue > range.max)) {
        node.warn(`Invalid ${fieldName} for ${deviceId}: ${scaledValue} (Out of range [${range.min}, ${range.max}])`);
        return null;
    }

    return scaledValue;
}

// --- Main Function Logic ---

const data = msg.payload;
const deviceId = msg.unitid ? `${deviceIdPrefix}${msg.unitid}` : `${deviceIdPrefix}1`;
const timestamp = Date.now();

const influxData = {
    measurement: influxMeasurement,
    tags: {
        device: deviceId
    },
    timestamp: timestamp,
    fields: {}
};

const fieldsToProcess = ["Temperature", "pH", "ORP", "motor_mA", "RPM"];

for (const field of fieldsToProcess) {
    const scaledValue = validateAndScale(data[field], field, deviceId);
    if (scaledValue !== null) {
        influxData.fields[field] = scaledValue;
    }
}

msg.payload = [influxData];
return msg;