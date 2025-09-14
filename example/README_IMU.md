# BNO055IMU Usage Guide

This document explains how to use the BNO055IMU sensor in FTC robot projects.

## File Overview

### 1. `imu_basic_example.java` - Basic IMU Example
The simplest IMU usage example, demonstrating:
- IMU declaration and initialization
- Reading basic orientation data (heading, pitch, roll)
- Displaying data on the driver station

**Perfect for beginners** to understand basic IMU concepts.

### 2. `imu_example.java` - Complete IMU Example
Full-featured IMU usage example, including:
- Complete IMU initialization and calibration
- Robot-oriented driving vs Field-oriented driving
- Using IMU for autonomous turning
- PID control algorithms for precise turning
- Detailed telemetry display

**Suitable for advanced users**, showcasing IMU applications in actual competition.

### 3. `ftc_sim_example.java` - Enhanced Original File
Enhanced your original code with IMU functionality:
- Added IMU initialization methods
- Integrated IMU data reading
- Added IMU information to existing telemetry display

## IMU Basic Concepts

### Axis Explanation
- **Heading (Z-axis)**: Robot's rotation angle, most important for navigation
- **Pitch (Y-axis)**: Robot's forward/backward tilt angle
- **Roll (X-axis)**: Robot's left/right tilt angle

### Angle Range
- All angles in degrees (-180° to +180°)
- Heading = 0°: Robot facing initial direction
- Heading = 90°: Robot turned 90 degrees left
- Heading = -90°: Robot turned 90 degrees right

## Code Usage Steps

### Step 1: Hardware Mapping
```java
BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
```

### Step 2: Initialize Parameters
```java
BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
imu.initialize(parameters);
```

### Step 3: Read Data
```java
Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
double heading = angles.firstAngle;   // Robot orientation
double pitch = angles.secondAngle;    // Forward/backward tilt
double roll = angles.thirdAngle;      // Left/right tilt
```

### Step 4: Display Data
```java
telemetry.addData("Heading", "%.1f degrees", heading);
telemetry.addData("Pitch", "%.1f degrees", pitch);
telemetry.addData("Roll", "%.1f degrees", roll);
telemetry.update();
```

## Real-World Applications

### 1. Field-Oriented Drive
Use IMU to make robot movement consistent relative to the field, rather than relative to the robot itself.

### 2. Autonomous Navigation
Use IMU for precise turning and positioning in autonomous mode.

### 3. Stability Control
Detect if the robot is tilting to prevent tipping over.

### 4. Path Tracking
Combine with other sensors to use IMU for complex path tracking.

## Important Notes

1. **Calibration is crucial**: IMU needs several seconds to calibrate, wait for calibration to complete before use
2. **Avoid magnetic interference**: Keep IMU away from large motors and metal structures
3. **Regular updates**: Read IMU data regularly in the main loop
4. **Angle normalization**: Limit angles to -180° to +180° range

## Hardware Configuration

Make sure to set up correctly in Robot Configuration:
- Sensor type: I2C Device
- Device name: "imu" (must match the name in code)
- I2C port: Set according to actual connection port

## Troubleshooting

If IMU doesn't work, check:
1. Hardware connections are correct
2. I2C port configuration matches
3. IMU is calibrated (`imu.isGyroCalibrated()`)
4. Device name in code matches configuration
