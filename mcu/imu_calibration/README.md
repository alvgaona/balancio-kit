# IMU Calibration Tool

This tool calibrates the MPU6050 IMU sensor on the Balancio board by finding optimal offsets for the accelerometer and gyroscope.

## Overview

The calibration process uses a PID-based algorithm to automatically determine the best offset values for your specific MPU6050 sensor. These offsets compensate for manufacturing variations and ensure accurate readings when the sensor is at rest.

## Hardware Requirements

- Balancio board with MPU6050 IMU
- USB cable for programming and serial monitoring
- A flat, stable surface for calibration

## Pin Configuration

The tool is configured for the Balancio board's I2C pins:
- SDA: GPIO 26
- SCL: GPIO 27

## Building and Uploading

1. Install PlatformIO (VS Code extension or CLI)
2. Open this project folder in PlatformIO
3. Build and upload:
   ```bash
   pio run -t upload
   ```

## Calibration Process

1. **Prepare the hardware:**
   - Place the Balancio board on a perfectly flat and horizontal surface
   - Ensure the board won't move during calibration
   - Let the board warm up for 5-10 minutes to stabilize temperature

2. **Run the calibration:**
   - Open the serial monitor at 9600 baud:
     ```bash
     pio device monitor -b 9600
     ```
   - Reset the board to start calibration
   - The process will take several minutes

3. **Understanding the output:**
   - The tool performs initial PID tuning (shown as dots, each dot = 100 readings)
   - It displays offset values at 600, 700, 800, 900, and 1000 readings
   - Then performs a more detailed calibration using binary search
   - The final line before "done" shows the optimal offsets

## Interpreting Results

The output format is:
```
[AxOffset,AxOffset] --> [AxMin,AxMax]  [AyOffset,AyOffset] --> [AyMin,AyMax]  [AzOffset,AzOffset] --> [AzMin,AzMax]
[GxOffset,GxOffset] --> [GxMin,GxMax]  [GyOffset,GyOffset] --> [GyMin,GyMax]  [GzOffset,GzOffset] --> [GzMin,GzMax]
```

Where:
- `AxOffset`, `AyOffset`, `AzOffset`: Accelerometer offsets for X, Y, Z axes
- `GxOffset`, `GyOffset`, `GzOffset`: Gyroscope offsets for X, Y, Z axes
- The arrow shows the resulting raw values with these offsets applied

Example output:
```
[567,567] --> [-1,2]  [-2223,-2223] --> [0,1]  [1131,1132] --> [16374,16404]
[155,156] --> [-1,1]  [-25,-24] --> [0,3]  [5,6] --> [0,4]
```

This means:
- X Accel offset: +567
- Y Accel offset: -2223
- Z Accel offset: +1131
- X Gyro offset: +155
- Y Gyro offset: -25
- Z Gyro offset: +5

## Using the Calibration Values

Once you have the offset values, use them in your main code:

```cpp
MPU6050 mpu;

void setup() {
    mpu.initialize();

    // Apply your calibration offsets
    mpu.setXAccelOffset(567);
    mpu.setYAccelOffset(-2223);
    mpu.setZAccelOffset(1131);
    mpu.setXGyroOffset(155);
    mpu.setYGyroOffset(-25);
    mpu.setZGyroOffset(5);
}
```

## Target Values

When properly calibrated, the sensor at rest should read:
- Accelerometer X, Y: 0
- Accelerometer Z: 16384 (1g in the sensor's scale)
- Gyroscope X, Y, Z: 0

## Troubleshooting

- **"MPU6050 connection failed"**: Check I2C connections and ensure the sensor is powered
- **Values don't converge**: Ensure the surface is perfectly level and the board is not moving
- **Inconsistent results**: Let the sensor warm up longer before calibration

## Notes

- Calibration values are specific to each individual sensor
- Re-calibration may be needed if the sensor is subjected to physical shock
- Temperature changes can affect calibration, so calibrate at your typical operating temperature
