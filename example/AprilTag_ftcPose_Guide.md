# FTC AprilTag ftcPose Parameters Guide

This document explains all the parameters available in the `ftcPose` object when detecting AprilTags in FTC robotics.

## Overview

When you detect an AprilTag, the `AprilTagDetection` object contains an `ftcPose` field that provides comprehensive position and orientation information about the tag relative to the robot's camera.

## Basic Usage

```java
// Get AprilTag detections
List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

for (AprilTagDetection detection : detections) {
    // Access ftcPose data
    double range = detection.ftcPose.range;
    double bearing = detection.ftcPose.bearing;
    double yaw = detection.ftcPose.yaw;
    // ... etc
}
```

## Position Parameters (Translation)

### 🎯 `x` - Lateral Position
- **Type**: `double` (inches)
- **Meaning**: Left/right position of the tag relative to camera
- **Positive**: Tag is to the RIGHT of the camera
- **Negative**: Tag is to the LEFT of the camera
- **Zero**: Tag is directly in front of the camera (centered horizontally)
- **Usage**: Strafing left/right to center on tag

```java
double x = detection.ftcPose.x;
if (x > 2) {
    // Tag is to the right, strafe left to center
} else if (x < -2) {
    // Tag is to the left, strafe right to center
}
```

### 📏 `y` - Vertical Position
- **Type**: `double` (inches)
- **Meaning**: Up/down position of the tag relative to camera
- **Positive**: Tag is ABOVE the camera level
- **Negative**: Tag is BELOW the camera level
- **Zero**: Tag is at the same height as camera
- **Usage**: Adjusting robot height or camera angle

```java
double y = detection.ftcPose.y;
if (y > 5) {
    // Tag is high, might need to raise mechanism
} else if (y < -5) {
    // Tag is low, might need to lower mechanism
}
```

### 🔍 `z` - Distance (Forward/Backward)
- **Type**: `double` (inches)
- **Meaning**: How far the tag is from the camera
- **Positive**: Tag is IN FRONT of the camera
- **Negative**: Tag is BEHIND the camera (rare)
- **Zero**: Tag is at camera position (impossible in practice)
- **Usage**: Moving forward/backward to reach target distance

```java
double z = detection.ftcPose.z;
if (z > 24) {
    // Tag is far, move forward
} else if (z < 12) {
    // Tag is close, move backward
}
```

## Convenient Distance Parameters

### 📐 `range` - Direct Distance
- **Type**: `double` (inches)
- **Meaning**: Straight-line distance from camera to tag
- **Formula**: `range = sqrt(x² + y² + z²)`
- **Always Positive**: Distance is always positive
- **Usage**: Overall proximity to tag, most commonly used

```java
double range = detection.ftcPose.range;
telemetry.addData("Distance to Tag", "%.1f inches", range);

if (range < 18) {
    // Close enough for precise operations
}
```

### 🧭 `bearing` - Horizontal Angle
- **Type**: `double` (degrees)
- **Meaning**: Horizontal angle from camera center to tag
- **Positive**: Tag is to the RIGHT
- **Negative**: Tag is to the LEFT
- **Zero**: Tag is straight ahead
- **Range**: Typically -180° to +180°
- **Usage**: Turning to face the tag

```java
double bearing = detection.ftcPose.bearing;
if (Math.abs(bearing) > 5) {
    // Turn to face the tag
    double turnPower = bearing * 0.02; // Simple proportional control
}
```

## Orientation Parameters (Rotation)

### 🔄 `yaw` - Tag Rotation (Z-axis)
- **Type**: `double` (degrees)
- **Meaning**: How much the tag is rotated around its vertical axis
- **Positive**: Tag rotated clockwise (from robot's perspective)
- **Negative**: Tag rotated counter-clockwise
- **Zero**: Tag facing directly toward robot
- **Usage**: Determining tag orientation for alignment

```java
double yaw = detection.ftcPose.yaw;
if (Math.abs(yaw) > 10) {
    // Tag is not facing us directly
    telemetry.addData("Tag Angle", "%.1f degrees", yaw);
}
```

### ⬆️ `pitch` - Tag Tilt (Y-axis)
- **Type**: `double` (degrees)
- **Meaning**: How much the tag is tilted up or down
- **Positive**: Top of tag tilted AWAY from robot
- **Negative**: Top of tag tilted TOWARD robot
- **Zero**: Tag is perfectly vertical
- **Usage**: Detecting if tag is mounted at an angle

```java
double pitch = detection.ftcPose.pitch;
if (Math.abs(pitch) > 15) {
    // Tag is significantly tilted
    telemetry.addData("Tag Tilt", "%.1f degrees", pitch);
}
```

### 🔃 `roll` - Tag Lean (X-axis)
- **Type**: `double` (degrees)
- **Meaning**: How much the tag is leaning left or right
- **Positive**: Tag leaning to robot's LEFT
- **Negative**: Tag leaning to robot's RIGHT
- **Zero**: Tag is perfectly upright
- **Usage**: Detecting tag mounting issues

```java
double roll = detection.ftcPose.roll;
if (Math.abs(roll) > 15) {
    // Tag is leaning significantly
    telemetry.addData("Tag Lean", "%.1f degrees", roll);
}
```

## Complete Example

```java
public void displayAprilTagData() {
    List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
    
    for (AprilTagDetection detection : detections) {
        telemetry.addData("Tag ID", detection.id);
        
        // Position data
        telemetry.addData("X (Left/Right)", "%.1f inches", detection.ftcPose.x);
        telemetry.addData("Y (Up/Down)", "%.1f inches", detection.ftcPose.y);
        telemetry.addData("Z (Distance)", "%.1f inches", detection.ftcPose.z);
        
        // Convenient distance/angle
        telemetry.addData("Range", "%.1f inches", detection.ftcPose.range);
        telemetry.addData("Bearing", "%.1f degrees", detection.ftcPose.bearing);
        
        // Orientation data
        telemetry.addData("Yaw (Rotation)", "%.1f degrees", detection.ftcPose.yaw);
        telemetry.addData("Pitch (Tilt)", "%.1f degrees", detection.ftcPose.pitch);
        telemetry.addData("Roll (Lean)", "%.1f degrees", detection.ftcPose.roll);
    }
    
    telemetry.update();
}
```

## Common Usage Patterns

### 🎯 Centering on Tag
```java
// Use x and bearing for horizontal alignment
double strafeCorrection = detection.ftcPose.x * 0.1;
double turnCorrection = detection.ftcPose.bearing * 0.02;
```

### 📏 Approaching Target Distance
```java
// Use range or z for distance control
double targetDistance = 18.0; // inches
double forwardCorrection = (detection.ftcPose.range - targetDistance) * 0.05;
```

### 🔍 Tag Quality Check
```java
// Check if tag detection is reliable
if (detection.ftcPose.range < 60 && 
    Math.abs(detection.ftcPose.yaw) < 45 &&
    Math.abs(detection.ftcPose.pitch) < 30) {
    // Good detection, safe to use for navigation
}
```

## Coordinate System

```
Camera View (Robot's Perspective):
         Y+ (Up)
          |
          |
    X-    |    X+ (Right)
   (Left) |
    ------+------
          |
          |
         Y- (Down)

Z+ = Forward (into the screen)
Z- = Backward (out of the screen)
```

## Tips for Better Detection

1. **Lighting**: Ensure good, even lighting on AprilTags
2. **Distance**: Best detection range is 1-4 feet
3. **Angle**: Tags work best when facing camera directly
4. **Size**: Larger tags = better detection at distance
5. **Stability**: Avoid camera shake for accurate readings

## Troubleshooting

- **Erratic readings**: Check camera mounting and stability
- **No detections**: Verify lighting and tag visibility
- **Wrong coordinates**: Check camera orientation and calibration
- **Jumpy values**: Add filtering/smoothing to readings
