# Limelight Camera Setup & Usage Guide for FTC

This guide covers setting up a Limelight 3A camera for FTC robotics and explains the code flow for AprilTag-based auto-drive and auto-aim.

---

## Table of Contents

1. [Hardware Setup](#1-hardware-setup)
2. [Software Configuration](#2-software-configuration)
3. [Robot Configuration](#3-robot-configuration)
4. [Code Flow Overview](#4-code-flow-overview)
5. [Auto-Drive Logic](#5-auto-drive-logic)
6. [Tuning Parameters](#6-tuning-parameters)
7. [Troubleshooting](#7-troubleshooting)

---

## 1. Hardware Setup

### 1.1 Mounting the Limelight

1. **Position**: Mount the Limelight on your robot where it has a clear view of AprilTags
   - Typically mounted on the front or back of the robot
   - Ensure it's secured firmly to avoid vibration during movement
   
2. **Orientation**: Note the camera's orientation relative to your robot
   - The Limelight's coordinate system matters for pose calculations
   - Document your mounting angle for later calibration

3. **Wiring**:
   ```
   Limelight 3A → Control Hub
   ─────────────────────────
   USB-C cable → USB port on Control Hub
   ```
   
   > **Note**: Limelight 3A uses USB connection. Make sure to use a quality USB-C cable.

### 1.2 Power Considerations

- Limelight 3A is powered via USB from the Control Hub
- Ensure your Control Hub has adequate power supply
- Consider using a powered USB hub if you have multiple USB devices

---

## 2. Software Configuration

### 2.1 Accessing the Limelight Web Interface

1. **Connect to Robot WiFi** (or connect via USB)

2. **Open Web Browser** and navigate to:
   ```
   http://limelight.local:5801
   ```
   
   > If `limelight.local` doesn't work, try the IP address (usually `10.TE.AM.11` where TEAM is your team number)

### 2.2 Configure AprilTag Pipeline

1. **Select Pipeline 0** (or create a new pipeline)

2. **Set Pipeline Type**: Select `AprilTag` or `Fiducial`

3. **Configure AprilTag Settings**:
   
   | Setting | Recommended Value | Description |
   |---------|-------------------|-------------|
   | Tag Family | `36h11` | Standard FTC AprilTag family |
   | Tag Size | `0.0508` meters (2 inches) | Size of FTC AprilTags |
   | Decimation | `2` | Balance between range and speed |

4. **Camera Settings**:
   
   | Setting | TeleOp | Autonomous |
   |---------|--------|------------|
   | Exposure | Higher (5-10ms) | Lower (2-4ms) |
   | Gain | Higher (200-250) | Lower (80-120) |
   
   > **Tip**: Lower exposure = faster detection but shorter range. Adjust based on your lighting conditions.

### 2.3 Camera Calibration (Optional but Recommended)

1. Go to **Settings → Calibration**
2. Follow the on-screen instructions with a calibration board
3. Save calibration for more accurate pose estimation

### 2.4 Save Configuration

1. Click **Save** to save pipeline settings
2. Set Pipeline 0 as the **default pipeline** if desired

---

## 3. Robot Configuration

### 3.1 Add Limelight to Hardware Map

1. Open **FTC Driver Station** or **Robot Controller**
2. Go to **Configure Robot**
3. Add a new device:
   - **Type**: `Limelight3A`
   - **Name**: `limelight`
   - **Port**: USB (should auto-detect)

### 3.2 Verify Connection

After configuring, the Limelight should show as connected in the hardware map. You can verify by checking the Limelight web interface - it should show "Connected" status.

---

## 4. Code Flow Overview

### 4.1 Initialization Flow

```
┌─────────────────────────────────────────────────────────────┐
│                      runOpMode()                             │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Initialize Hardware                                      │
│     ├── Drive motors (fl, fr, bl, br)                       │
│     ├── IMU                                                  │
│     ├── Accessory motors                                     │
│     └── Limelight camera ─────────────────┐                 │
│                                            │                 │
│                                            ▼                 │
│                               ┌─────────────────────┐       │
│                               │   initLimelight()   │       │
│                               ├─────────────────────┤       │
│                               │ 1. Get from hardwareMap     │
│                               │ 2. Switch to pipeline 0     │
│                               │ 3. Start polling            │
│                               └─────────────────────┘       │
│                                                              │
│  2. waitForStart()                                          │
│                                                              │
│  3. Main Loop ──────────────────────────────────────────────│
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Main Loop Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    MAIN LOOP (while opModeIsActive)          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Check A Button Pressed?                  │   │
│  └──────────────────┬───────────────────────────────────┘   │
│                     │                                        │
│         ┌───────────┴───────────┐                           │
│         │                       │                           │
│         ▼                       ▼                           │
│   ┌──────────┐           ┌──────────────┐                   │
│   │ A = YES  │           │   A = NO     │                   │
│   └────┬─────┘           └──────┬───────┘                   │
│        │                        │                           │
│        ▼                        ▼                           │
│  ┌─────────────┐         ┌─────────────────┐               │
│  │ Auto-Drive  │         │  Manual Control │               │
│  │    Mode     │         │      Mode       │               │
│  └─────────────┘         └─────────────────┘               │
│        │                        │                           │
│        ▼                        ▼                           │
│  ┌─────────────┐         ┌─────────────────┐               │
│  │detectAprilTag()       │ Read gamepad    │               │
│  └─────────────┘         │ joysticks       │               │
│        │                  └─────────────────┘               │
│        ▼                        │                           │
│  ┌─────────────────────┐       │                           │
│  │getAutoDriveCommands()│       │                           │
│  └─────────────────────┘       │                           │
│        │                        │                           │
│        └────────────┬───────────┘                           │
│                     │                                        │
│                     ▼                                        │
│           ┌─────────────────┐                               │
│           │ Apply motor     │                               │
│           │ powers (mecanum)│                               │
│           └─────────────────┘                               │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 4.3 Code Structure

```java
// Main components in TeleOpBlue.java

class TeleOpBlue {
    
    // Limelight variables
    private Limelight3A limelight;                    // Camera object
    private LLResultTypes.FiducialResult desiredFiducial;  // Detected tag
    
    // Methods
    void initLimelight()           // Initialize camera
    boolean detectAprilTag()       // Find target AprilTag
    void displayAprilTagStatus()   // Show telemetry
    double[] getAutoDriveCommands() // Calculate drive commands
}
```

---

## 5. Auto-Drive Logic

### 5.1 Detection Flow

```
┌─────────────────────────────────────────────────────────────┐
│                   detectAprilTag()                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   1. Get latest result from Limelight                       │
│      └── LLResult result = limelight.getLatestResult()      │
│                                                              │
│   2. Check if result is valid                               │
│      └── result != null && result.isValid()                 │
│                                                              │
│   3. Get fiducial (AprilTag) results                        │
│      └── result.getFiducialResults()                        │
│                                                              │
│   4. Find desired tag ID                                    │
│      └── Loop through fiducials                             │
│          └── Check if fiducial.getFiducialId() == target    │
│                                                              │
│   5. Return true if found, false otherwise                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Auto-Drive Calculation

The auto-drive system uses three control loops:

```
┌─────────────────────────────────────────────────────────────┐
│              getAutoDriveCommands() - Control Loops          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ 1. FORWARD/BACKWARD (Range Control)                  │    │
│  │                                                      │    │
│  │    rangeError = currentRange - DESIRED_DISTANCE      │    │
│  │    fwd = rangeError × SPEED_GAIN                     │    │
│  │                                                      │    │
│  │    Example: Tag at 80", target 60"                   │    │
│  │    → error = 20" → fwd = 0.7 (drive forward)        │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ 2. STRAFE (Lateral Alignment)                        │    │
│  │                                                      │    │
│  │    yawError = robotYaw - YAW_OFFSET                  │    │
│  │    str = yawError × SPEED_GAIN                       │    │
│  │                                                      │    │
│  │    Example: Robot 10° right of center                │    │
│  │    → error = 10° → str = 0.35 (strafe left)         │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ 3. TURN (Heading Correction)                         │    │
│  │                                                      │    │
│  │    headingError = bearing - BEARING_OFFSET           │    │
│  │    yaw = -headingError × TURN_GAIN                   │    │
│  │                                                      │    │
│  │    Example: Tag 15° to the right                     │    │
│  │    → error = 15° → yaw = -0.25 (turn right)         │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 5.3 Pose Data from Limelight

```
┌─────────────────────────────────────────────────────────────┐
│              Limelight Pose Data Structure                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  desiredFiducial.getRobotPoseTargetSpace()                  │
│       │                                                      │
│       └── Pose3D                                            │
│            ├── Position (x, y, z in meters)                 │
│            │    ├── x: lateral offset                       │
│            │    ├── y: vertical offset                      │
│            │    └── z: distance to target                   │
│            │                                                │
│            └── Orientation (YawPitchRoll)                   │
│                 ├── yaw: rotation around vertical           │
│                 ├── pitch: rotation around lateral          │
│                 └── roll: rotation around forward           │
│                                                              │
│  desiredFiducial.getTargetXDegrees()                        │
│       └── Horizontal angle to target (bearing)              │
│                                                              │
│  desiredFiducial.getTargetYDegrees()                        │
│       └── Vertical angle to target                          │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 5.4 Visual Representation

```
                    AprilTag
                       │
                       │ Range (z)
                       │
                       ▼
        ◄──── Yaw ────►
                    ┌─────┐
                    │     │
                    │Robot│ ──── Bearing ────►
                    │     │
                    └─────┘
                       │
                       │
                    Forward
```

---

## 6. Tuning Parameters

### 6.1 Control Gains

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `SPEED_GAIN` | 0.035 | 0.01-0.1 | Higher = faster approach, more overshoot |
| `TURN_GAIN` | 0.017 | 0.01-0.05 | Higher = faster turning, more oscillation |
| `MAX_AUTO_SPEED` | 0.55 | 0.3-0.8 | Maximum forward/strafe speed |
| `MAX_AUTO_TURN` | 0.25 | 0.1-0.4 | Maximum turn speed |

### 6.2 Target Settings

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DESIRED_DISTANCE` | 60.0 | Target distance from AprilTag (inches) |
| `DESIRED_TAG_ID` | 20 | AprilTag ID to track (-1 for any tag) |
| `BEARING_OFFSET` | 0.0 | Compensate for camera not centered |
| `YAW_OFFSET` | 0.0 | Compensate for camera angle offset |

### 6.3 Tuning Process

1. **Start with low gains** (SPEED_GAIN = 0.02, TURN_GAIN = 0.01)
2. **Test range control** first - does robot approach target distance?
3. **Increase SPEED_GAIN** until robot responds quickly but doesn't overshoot
4. **Test turning** - does robot face the tag?
5. **Increase TURN_GAIN** until robot aligns without oscillating
6. **Adjust offsets** if robot consistently stops off-center

---

## 7. Troubleshooting

### 7.1 Common Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| "No result" in telemetry | Limelight not connected | Check USB cable, restart robot |
| Tag not detected | Wrong pipeline | Switch pipeline with UP/DOWN |
| Tag not detected | Exposure too low | Increase exposure in Limelight UI |
| Robot overshoots | Gains too high | Reduce SPEED_GAIN or TURN_GAIN |
| Robot oscillates | TURN_GAIN too high | Reduce TURN_GAIN |
| Robot stops off-center | Camera offset | Adjust BEARING_OFFSET and YAW_OFFSET |
| Pose data is null | Tag too far | Move closer or reduce decimation |

### 7.2 Debug Telemetry

The code displays useful debug information:

```
DEBUG Range: 65.2" (target: 60.0")
DEBUG Yaw: 5.3° (offset: 0.0°, error: 5.3°)  
DEBUG Bearing: -3.2° (offset: 0.0°, error: -3.2°)
```

### 7.3 Pipeline Switching

Use **UP/DOWN** buttons on gamepad to switch between pipelines (0-9):
- **UP**: Next pipeline (higher number)
- **DOWN**: Previous pipeline (lower number)

This is useful for:
- Different lighting conditions
- Different detection modes
- Testing configurations

---

## Quick Reference

### Controls

| Button | Function |
|--------|----------|
| A | Hold for AprilTag auto-drive |
| B | Toggle field-centric/robot-centric |
| Y | Reset IMU heading |
| UP | Switch to next pipeline |
| DOWN | Switch to previous pipeline |

### Key Methods

| Method | Purpose |
|--------|---------|
| `initLimelight()` | Initialize Limelight camera |
| `detectAprilTag()` | Search for target AprilTag |
| `displayAprilTagStatus()` | Show detection info on telemetry |
| `getAutoDriveCommands()` | Calculate drive powers from pose |

---

## Resources

- [Limelight Documentation](https://docs.limelightvision.io/)
- [FTC Limelight API](https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html)
- [AprilTag Family Info](https://april.eecs.umich.edu/software/apriltag)

