# IMU Calibration Algorithm Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Background: Why Calibration is Necessary](#background-why-calibration-is-necessary)
3. [Algorithm Overview](#algorithm-overview)
4. [Phase 1: PID-Based Initial Calibration](#phase-1-pid-based-initial-calibration)
5. [Phase 2: Binary Search Refinement](#phase-2-binary-search-refinement)
6. [Mathematical Foundation](#mathematical-foundation)
7. [Implementation Details](#implementation-details)
8. [Convergence Criteria](#convergence-criteria)
9. [Performance Considerations](#performance-considerations)

## Introduction

The MPU6050 calibration algorithm is a sophisticated two-phase approach that combines PID (Proportional-Integral-Derivative) control theory with binary search optimization to find optimal offset values for the accelerometer and gyroscope sensors.

## Background: Why Calibration is Necessary

### Manufacturing Variations
Every MEMS (Micro-Electro-Mechanical Systems) sensor has slight manufacturing variations that cause:
- **Zero-point offset**: The sensor doesn't read exactly zero when stationary
- **Scale factor errors**: The sensor's sensitivity may deviate from the nominal value
- **Cross-axis sensitivity**: Movement in one axis affects readings in other axes
- **Temperature drift**: Sensor characteristics change with temperature

### Ideal vs. Real Sensor Behavior

**Ideal MPU6050 at rest:**
- Accelerometer X, Y: 0 LSB (Least Significant Bits)
- Accelerometer Z: 16384 LSB (equivalent to 1g at ±2g range)
- Gyroscope X, Y, Z: 0 LSB

**Real MPU6050 without calibration:**
- May show significant non-zero values even when perfectly still
- Z-axis acceleration may not equal 16384 LSB
- Gyroscope may indicate rotation when stationary

## Algorithm Overview

The calibration process consists of two main phases:

1. **PID-Based Rapid Estimation** (First ~10 seconds)
   - Uses control theory to quickly converge on approximate offsets
   - Provides 5 sets of progressively refined estimates
   - Offers fast, reasonably accurate results
   - **Note**: In the actual implementation, this phase is handled by the MPU6050 library's `CalibrateAccel()` and `CalibrateGyro()` methods

2. **Binary Search Refinement** (Following ~5-10 minutes)
   - Uses bracketing and binary search for precise optimization
   - Achieves maximum possible accuracy
   - Handles both "fast" (1000 samples) and "slow" (10000 samples) modes
   - **Note**: This phase is directly implemented in `main.cpp`

## Phase 1: PID-Based Initial Calibration

### Implementation Note

In the Balancio IMU calibration tool, the PID algorithm is abstracted within the MPU6050 library. The `main.cpp` file calls:
```cpp
accelgyro.CalibrateAccel(6);  // 600 readings
accelgyro.CalibrateGyro(6);
```
These methods internally implement the PID algorithm described below. The following explanation is provided for understanding the underlying algorithm.

### PID Control Theory Application

The algorithm uses PI control (Proportional-Integral, without Derivative) because:
- **Proportional (P)**: Provides immediate response to error
- **Integral (I)**: Accumulates error over time, eliminating steady-state offset
- **Derivative (D)**: Omitted due to sensor noise

### Mathematical Model

For each axis, the PI controller operates as:

```
error[n] = target - measured[n]
integral[n] = integral[n-1] + (error[n] * Ki)
offset[n] = integral[n] + (error[n] * Kp)
```

Where:
- `target`: Desired sensor reading (0 for most axes, 16384 for Z-accel)
- `measured[n]`: Current sensor reading
- `Ki`: Integral gain (determines convergence speed)
- `Kp`: Proportional gain (reduces noise influence)

### Implementation Details

The PID phase performs 5 iterations:
1. **600 readings**: Initial rough estimate
2. **700 readings**: First refinement
3. **800 readings**: Second refinement
4. **900 readings**: Third refinement
5. **1000 readings**: Final PID estimate

In `main.cpp`, this is implemented as:
```cpp
accelgyro.CalibrateAccel(6);  // 600 readings
accelgyro.CalibrateGyro(6);
accelgyro.PrintActiveOffsets();

accelgyro.CalibrateAccel(1);  // +100 = 700 total
accelgyro.CalibrateGyro(1);
// ... repeated to reach 1000 readings
```

Each iteration:
- Reads sensor data continuously
- Updates the integral term with each reading
- Applies new offsets to the sensor
- The integral naturally converges to the required offset

### Advantages of PID Approach
- **Fast convergence**: Provides usable results in seconds
- **Adaptive**: Automatically adjusts based on error magnitude
- **Noise filtering**: Integral action naturally filters high-frequency noise

## Phase 2: Binary Search Refinement

### Bracketing Algorithm

The binary search phase is fully implemented in `main.cpp` and uses a two-step approach:

1. **Bracket Expansion** (`PullBracketsOut()`)
   - Finds offset values that bracket the target
   - Ensures Low < Target < High for each axis
   - Expands by 1000 LSB steps until brackets are valid

2. **Bracket Contraction** (`PullBracketsIn()`)
   - Uses binary search to narrow the brackets
   - Switches from "fast" to "slow" sampling when brackets are narrow
   - Continues until brackets cannot be reduced further

### Binary Search Mathematics

For each axis, the algorithm maintains:
- `LowOffset[i]`: Offset that produces reading below target
- `HighOffset[i]`: Offset that produces reading above target
- `LowValue[i]`: Sensor reading with LowOffset
- `HighValue[i]`: Sensor reading with HighOffset

The search process:
```
while (HighOffset[i] > LowOffset[i] + 1) {
    MidOffset[i] = (LowOffset[i] + HighOffset[i]) / 2
    MidValue[i] = ReadSensorWithOffset(MidOffset[i])

    if (MidValue[i] > Target[i]) {
        HighOffset[i] = MidOffset[i]
        HighValue[i] = MidValue[i]
    } else {
        LowOffset[i] = MidOffset[i]
        LowValue[i] = MidValue[i]
    }
}
```

### Sampling Strategy

The algorithm uses two sampling modes:

**Fast Mode** (NFast = 1000 samples):
- Used during initial bracketing
- Provides quick feedback
- ~5 seconds per measurement

**Slow Mode** (NSlow = 10000 samples):
- Activated when all brackets are narrow (<10 LSB)
- Provides maximum accuracy
- ~50 seconds per measurement

## Mathematical Foundation

### Sensor Model

The MPU6050 output can be modeled as:
```
Output = Scale * (TrueValue + Bias) + Offset + Noise
```

Where:
- `Scale`: Sensitivity (LSB per unit)
- `TrueValue`: Actual physical quantity
- `Bias`: Inherent sensor bias
- `Offset`: User-programmable offset (what we're calibrating)
- `Noise`: Random measurement noise

### Calibration Goal

Find `Offset` such that:
```
E[Output] = Target
```

Where `E[]` denotes expected value (average).

### Noise Reduction Through Averaging

The algorithm samples at ~200 Hz (5ms intervals) and averages:
```
Smoothed[i] = (Σ RawValue[i]) / N
```

This reduces noise by a factor of √N:
- Fast mode (N=1000): ~31.6x noise reduction
- Slow mode (N=10000): ~100x noise reduction

## Implementation Details

### Code Structure

The `main.cpp` file contains:
- **Initialize()**: Sets up I2C, MPU6050, and runs PID calibration via library calls
- **GetSmoothed()**: Averages N sensor readings (used in binary search phase)
- **PullBracketsOut()**: Expands search brackets
- **PullBracketsIn()**: Binary search refinement
- **SetOffsets()**: Applies offsets to sensor registers

### Key Constants

```cpp
const int usDelay = 3150;        // Microsecond delay for ~200 Hz sampling
const int NFast = 1000;          // Samples for fast mode
const int NSlow = 10000;         // Samples for slow mode
const int Target[6] = {0, 0, 16384, 0, 0, 0};  // Target values
```

### I2C Communication

The algorithm configures I2C for 400 kHz operation:
```cpp
Wire.begin(SDA, SCL, 400000);
```

This ensures rapid sensor communication without bottlenecks.

### Memory Efficiency

The algorithm uses:
- 6 integer arrays for offsets and values
- Long integers for accumulation (prevents overflow)
- Minimal dynamic memory allocation

## Convergence Criteria

### PID Phase Convergence

The PID phase doesn't check for convergence explicitly but relies on:
- Integral wind-up limitation through offset register limits
- Natural convergence of the integral term
- Fixed iteration count (5 rounds)

### Binary Search Convergence

Convergence occurs when:
```
For all axes i:
    HighOffset[i] <= LowOffset[i] + 1
```

This means the bracket cannot be reduced further without losing the target containment.

### Accuracy Expectations

**Theoretical accuracy:**
- ±1 LSB for offset determination
- ±0.06% of full scale for accelerometer (±2g range)
- ±0.008°/s for gyroscope (±250°/s range)

**Practical accuracy:**
- Accelerometer: ±0.01g typical
- Gyroscope: ±0.1°/s typical
- Limited by sensor noise and environmental factors

## Performance Considerations

### Time Complexity

**PID Phase:**
- Fixed time: ~10 seconds
- O(1) regardless of initial offset

**Binary Search Phase:**
- Bracket expansion: O(k) where k = distance to valid bracket / 1000
- Bracket contraction: O(log(range)) per axis
- Total: ~5-10 minutes typical

### Sampling Rate Impact

The 200 Hz sampling rate is chosen to:
- Stay well above sensor bandwidth (5-260 Hz configurable)
- Avoid aliasing of mechanical vibrations
- Provide sufficient samples within reasonable time
- Prevent I2C bus saturation

### Temperature Considerations

For best results:
1. Allow 5-10 minute warm-up before calibration
2. Calibrate at typical operating temperature
3. Consider temperature compensation for critical applications

### Environmental Requirements

The algorithm assumes:
- Stable, level surface (±0.1° ideal)
- Minimal vibration
- Constant temperature during calibration
- Local gravity = 9.80665 m/s² (standard gravity)

## Algorithm Limitations

1. **Single-point calibration**: Only calibrates offset, not scale or linearity
2. **Temperature dependence**: Calibration is valid for a specific temperature
3. **Aging effects**: Sensor characteristics may drift over time
4. **Cross-axis coupling**: Not addressed by this algorithm
5. **Assumes perfect 1g**: Doesn't account for local gravity variations

## Advanced Considerations

### Potential Improvements

1. **Temperature compensation**: Store offsets at multiple temperatures
2. **Scale factor calibration**: Use multiple orientations
3. **Cross-axis compensation**: Full calibration matrix
4. **Adaptive sampling**: Adjust N based on noise level
5. **Quality metrics**: Report confidence in calibration

### Integration with Applications

Calibrated offsets should be:
1. Stored in non-volatile memory (EEPROM/Flash)
2. Applied during sensor initialization
3. Verified periodically
4. Re-calibrated after physical shocks

## Conclusion

This two-phase calibration algorithm provides an excellent balance between speed and accuracy. The PID phase offers quick results suitable for many applications, while the binary search phase achieves maximum possible accuracy for demanding uses. The combination ensures robust calibration across a wide range of initial conditions and sensor variations.
