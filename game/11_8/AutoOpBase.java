package org.firstinspires.ftc.teamcode.error404;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Base class for autonomous op modes with AprilTag support.
 * Contains shared AprilTag functionality and common drive/pose methods.
 * 
 * Child classes must implement:
 * - getDesiredTagId() to return the AprilTag ID for their alliance (20 for Blue, 24 for Red)
 */
public abstract class AutoOpBase extends LinearOpMode {
    // ---- Odometry constants ----
    protected static final double WHEEL_RADIUS_IN = 1.8898;
    protected static final double GEAR_RATIO = 1.0;
    protected static final int TICKS_PER_REV = 537;
    protected static final double INCHES_PER_TICK =
            (2 * Math.PI * WHEEL_RADIUS_IN * GEAR_RATIO) / TICKS_PER_REV;
    protected static double LATERAL_MULTIPLIER = 1.05;

    // ---- Autonomous driving constants ----
    protected static final double DRIVE_SPEED = 0.6;
    protected static final double TURN_SPEED = 0.8;
    protected static final double HEADING_THRESHOLD = 2.0;
    protected static final double DISTANCE_THRESHOLD = 2.0;
    
    // ---- Drive motors ----
    protected DcMotorEx fl, fr, bl, br;
    protected IMU imu;

    // ---- Pose tracking ----
    protected double x = 0, y = 0, heading = 0;
    protected double lastImuYaw = 0;
    protected int lastFL, lastFR, lastBL, lastBR;
    
    // ---- AprilTag variables ----
    protected final double DESIRED_DISTANCE = 65.0;
    protected final double SPEED_GAIN = 0.035;
    protected final double TURN_GAIN = 0.017;
    protected final double MAX_AUTO_SPEED = 0.55;
    protected final double MAX_AUTO_TURN = 0.25;
    protected final double BEARING_OFFSET = 0.0;
    protected final double YAW_OFFSET = 0.0;
    protected static final boolean USE_WEBCAM = true;
    
    // ---- Camera exposure settings ----
    // Six levels from bright (light_0) to dim (light_5)
    // Looped from lowest to highest (bright to dim)
    // light_0 = bright (2ms, 80 gain) - min
    // light_1 through light_4 = intermediate values with granularity
    // light_5 = dim (6ms, 250 gain) - max
    protected static final int[] EXPOSURE_LEVELS_MS = {2, 3, 4, 4, 4, 4, 4, 5};  // From bright to dim
    protected static final int[] GAIN_LEVELS = {80, 100, 120, 140, 160, 180, 200, 250};  // From bright to dim (evenly spaced)
    protected static final String[] LEVEL_NAMES = {"Bright", "Light-1", "Light-2", "Light-3", "Light-4", "Light-5", "Light-6", "Dim"};
    
    // Default exposure (bright - light_0)
    protected static final int DEFAULT_EXPOSURE_INDEX = 0;  // Index 0 = 2ms, 80 gain
    protected VisionPortal visionPortal;
    protected AprilTagProcessor aprilTag;
    protected AprilTagDetection desiredTag = null;
    
    // ---- Status tracking ----
    protected String currentStatus = "Initializing";
    
    // ---- Cached exposure setting ----
    // Stores the last successful exposure index to reuse on subsequent detections
    protected Integer cachedExposureIndex = null;

    /**
     * Get the desired AprilTag ID for this alliance.
     * Blue alliance should return 20, Red alliance should return 24.
     * 
     * @return The AprilTag ID to track
     */
    protected abstract int getDesiredTagId();

    // ==================== Pose Tracking ====================
    
    protected void updatePose() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double imuYaw = -ypr.getYaw(AngleUnit.RADIANS);  // Negate due to USB facing BACKWARD orientation
        double dTheta = wrap(imuYaw - lastImuYaw);
        lastImuYaw = imuYaw;

        int cFL = fl.getCurrentPosition(), cFR = fr.getCurrentPosition();
        int cBL = bl.getCurrentPosition(), cBR = br.getCurrentPosition();
        double dFL = (cFL - lastFL) * INCHES_PER_TICK;
        double dFR = (cFR - lastFR) * INCHES_PER_TICK;
        double dBL = (cBL - lastBL) * INCHES_PER_TICK;
        double dBR = (cBR - lastBR) * INCHES_PER_TICK;
        lastFL = cFL; lastFR = cFR; lastBL = cBL; lastBR = cBR;

        double dxR = (dFL + dFR + dBL + dBR) / 4.0;
        double dyR = (dFL - dFR - dBL + dBR) / 4.0 * LATERAL_MULTIPLIER;

        double hMid = heading + dTheta / 2.0;
        double c = Math.cos(hMid), s = Math.sin(hMid);
        x += dxR * c - dyR * s;
        y += dxR * s + dyR * c;
        heading = wrap(heading + dTheta);
    }
    
    // ==================== Drive Control ====================
    
    protected void setDrivePower(double fwd, double str, double yaw) {
        double flPow = fwd + str + yaw;
        double blPow = fwd - str + yaw;
        double frPow = fwd - str - yaw;
        double brPow = fwd + str - yaw;

        double max = Math.max(1.0, Math.max(Math.abs(flPow),
                    Math.max(Math.abs(blPow), Math.max(Math.abs(frPow), Math.abs(brPow)))));
        flPow /= max;
        blPow /= max;
        frPow /= max;
        brPow /= max;

        fl.setPower(flPow);
        bl.setPower(blPow);
        fr.setPower(frPow);
        br.setPower(brPow);
    }
    
    protected void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    
    protected static double wrap(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }
    
    // ==================== AprilTag Functions ====================
    
    protected void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
            // Set default exposure automatically (bright - light_0)
            setManualExposure(EXPOSURE_LEVELS_MS[DEFAULT_EXPOSURE_INDEX], GAIN_LEVELS[DEFAULT_EXPOSURE_INDEX]);
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /**
     * Set camera exposure and gain manually.
     * Lower values work better in bright light, higher values in dim light.
     * 
     * @param exposureMS Exposure time in milliseconds (typically 1-10ms)
     * @param gain Camera gain (typically 50-250)
     */
    protected void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }
        
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }
        
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    
    /**
     * Detect AprilTag with retry logic using different exposure settings.
     * Tries 6 exposure levels from bright (light_0) to dim (light_5).
     * Caches the successful exposure setting and reuses it on subsequent calls.
     * 
     * @return true if AprilTag was found, false otherwise
     */
    protected boolean detectAprilTag() {
        desiredTag = null;
        int desiredTagId = getDesiredTagId();
        
        telemetry.addLine("🔍 Detecting AprilTag...");
        telemetry.update();
        
        // First, try the cached successful exposure setting if available
        if (cachedExposureIndex != null) {
            int i = cachedExposureIndex;
            int exposureMS = EXPOSURE_LEVELS_MS[i];
            int gain = GAIN_LEVELS[i];
            String levelName = LEVEL_NAMES[i];
            
            telemetry.addLine(String.format("  Trying cached %s (light_%d: %dms, gain %d)...", 
                    levelName, i, exposureMS, gain));
            telemetry.update();
            
            if (tryDetectWithExposure(desiredTagId, exposureMS, gain, levelName)) {
                telemetry.addLine(String.format("✓ Found with cached %s (light_%d)!", levelName, i));
                telemetry.update();
                return true;
            } else {
                telemetry.addLine("  Cached setting failed, trying all levels...");
                telemetry.update();
                // Clear cache if it no longer works
                cachedExposureIndex = null;
            }
        }
        
        // Loop through all exposure levels from bright (index 0) to dim (index 5)
        for (int i = 0; i < EXPOSURE_LEVELS_MS.length; i++) {
            int exposureMS = EXPOSURE_LEVELS_MS[i];
            int gain = GAIN_LEVELS[i];
            String levelName = LEVEL_NAMES[i];
            
            telemetry.addLine(String.format("  Trying %s (light_%d: %dms, gain %d)...", 
                    levelName, i, exposureMS, gain));
            telemetry.update();
            
            if (tryDetectWithExposure(desiredTagId, exposureMS, gain, levelName)) {
                telemetry.addLine(String.format("✓ Found with %s (light_%d)!", levelName, i));
                telemetry.update();
                // Cache the successful exposure index for future use
                cachedExposureIndex = i;
                return true;
            }
        }
        
        telemetry.addLine("✗ No AprilTag found with any exposure setting");
        telemetry.update();
        return false;
    }
    
    /**
     * Try to detect AprilTag with a specific exposure setting.
     * 
     * @param desiredTagId The tag ID to look for
     * @param exposureMS Exposure time in milliseconds
     * @param gain Camera gain
     * @param settingName Name of the setting (for telemetry)
     * @return true if tag was found, false otherwise
     */
    private boolean tryDetectWithExposure(int desiredTagId, int exposureMS, int gain, String settingName) {
        // Set exposure for this attempt
        setManualExposure(exposureMS, gain);
        
        // Wait a bit for exposure to take effect
        sleep(100);
        
        // Try to detect
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    desiredTag = detection;
                    telemetry.addData("📸 Working Exposure", settingName + " (%dms, gain %d)", exposureMS, gain);
                    telemetry.update();
                    return true;
                }
            }
        }
        
        return false;
    }
    
    /**
     * Scan for AprilTag by slowly turning left and right
     * Continuously checks for AprilTag while rotating
     * 
     * @param degree The angle in degrees to scan in each direction
     * @param leftFirst If true, scan left first; if false, scan right first
     * @param timeoutSeconds Maximum time to attempt the scan
     * @return true if AprilTag was found, false otherwise
     */
    protected boolean scanForAprilTag(double degree, boolean leftFirst, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        final double SCAN_SPEED = 0.2;  // Slow turning speed for scanning
        
        // Record starting heading
        double startHeadingDegrees = Math.toDegrees(heading);
        
        telemetry.addLine("🔍 Scanning for AprilTag...");
        telemetry.addData("Scan Degree", "%.1f°", degree);
        telemetry.addData("Direction", leftFirst ? "LEFT first" : "RIGHT first");
        telemetry.update();
        
        // Define first and second directions based on leftFirst parameter
        String firstDirection = leftFirst ? "LEFT" : "RIGHT";
        String secondDirection = leftFirst ? "RIGHT" : "LEFT";
        double firstTurnSpeed = leftFirst ? -SCAN_SPEED : SCAN_SPEED;
        double secondTurnSpeed = leftFirst ? SCAN_SPEED : -SCAN_SPEED;
        
        // Phase 1: Turn in first direction
        double firstTarget = leftFirst ? (startHeadingDegrees - degree) : (startHeadingDegrees + degree);
        telemetry.addLine("🔍 Scanning " + firstDirection + "...");
        telemetry.update();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds / 4) {
            updatePose();
            
            // Check for AprilTag
            if (detectAprilTag()) {
                stopDrive();
                telemetry.addLine("✓ AprilTag FOUND during " + firstDirection + " scan!");
                telemetry.update();
                sleep(100);
                return true;
            }
            
            // Calculate heading error
            double currentHeadingDegrees = Math.toDegrees(heading);
            double headingError = wrap(Math.toRadians(firstTarget - currentHeadingDegrees));
            
            // Check if we've reached the first direction limit
            if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                break;
            }
            
            // Turn in first direction
            setDrivePower(0, 0, firstTurnSpeed);
            
            telemetry.addData("Status", currentStatus);
            telemetry.addData("🔍 Scanning", firstDirection);
            telemetry.addData("🧭 Current", "%.1f°", currentHeadingDegrees);
            telemetry.addData("🎯 Target", "%.1f°", firstTarget);
            telemetry.addData("AprilTag", "Not found yet...");
            telemetry.update();
            
            sleep(50);
        }
        
        stopDrive();
        sleep(100);
        
        // Phase 2: Turn back to start
        telemetry.addLine("🔍 Returning to center...");
        telemetry.update();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds / 4) {
            updatePose();
            
            // Check for AprilTag
            if (detectAprilTag()) {
                stopDrive();
                telemetry.addLine("✓ AprilTag FOUND while returning to center!");
                telemetry.update();
                sleep(100);
                return true;
            }
            
            // Calculate heading error
            double currentHeadingDegrees = Math.toDegrees(heading);
            double headingError = wrap(Math.toRadians(startHeadingDegrees - currentHeadingDegrees));
            
            // Check if we've reached the start
            if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                break;
            }
            
            // Turn back to start
            setDrivePower(0, 0, -firstTurnSpeed);
            
            telemetry.addData("Status", currentStatus);
            telemetry.addData("🔍 Scanning", "Returning to center");
            telemetry.addData("🧭 Current", "%.1f°", currentHeadingDegrees);
            telemetry.addData("🎯 Target", "%.1f°", startHeadingDegrees);
            telemetry.addData("AprilTag", "Not found yet...");
            telemetry.update();
            
            sleep(50);
        }
        
        stopDrive();
        sleep(100);
        
        // Phase 3: Turn in second direction (opposite of first)
        double secondTarget = leftFirst ? (startHeadingDegrees + degree) : (startHeadingDegrees - degree);
        telemetry.addLine("🔍 Scanning " + secondDirection + "...");
        telemetry.update();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds / 4) {
            updatePose();
            
            // Check for AprilTag
            if (detectAprilTag()) {
                stopDrive();
                telemetry.addLine("✓ AprilTag FOUND during " + secondDirection + " scan!");
                telemetry.update();
                sleep(100);
                return true;
            }
            
            // Calculate heading error
            double currentHeadingDegrees = Math.toDegrees(heading);
            double headingError = wrap(Math.toRadians(secondTarget - currentHeadingDegrees));
            
            // Check if we've reached the second direction limit
            if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                break;
            }
            
            // Turn in second direction
            setDrivePower(0, 0, secondTurnSpeed);
            
            telemetry.addData("Status", currentStatus);
            telemetry.addData("🔍 Scanning", secondDirection);
            telemetry.addData("🧭 Current", "%.1f°", currentHeadingDegrees);
            telemetry.addData("🎯 Target", "%.1f°", secondTarget);
            telemetry.addData("AprilTag", "Not found yet...");
            telemetry.update();
            
            sleep(50);
        }
        
        stopDrive();
        sleep(100);
        
        // Phase 4: Return to original heading
        telemetry.addLine("❌ No AprilTag found - returning to start...");
        telemetry.update();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds / 4) {
            updatePose();
            
            // Check for AprilTag even while returning
            if (detectAprilTag()) {
                stopDrive();
                telemetry.addLine("✓ AprilTag FOUND while returning to start!");
                telemetry.update();
                sleep(100);
                return true;
            }
            
            // Calculate heading error
            double currentHeadingDegrees = Math.toDegrees(heading);
            double headingError = wrap(Math.toRadians(startHeadingDegrees - currentHeadingDegrees));
            
            // Check if we've reached the start
            if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                break;
            }
            
            // Turn back to start
            setDrivePower(0, 0, -secondTurnSpeed);
            
            telemetry.addData("Status", currentStatus);
            telemetry.addData("🔍 Scanning", "Final return to start");
            telemetry.addData("🧭 Current", "%.1f°", currentHeadingDegrees);
            telemetry.addData("🎯 Target", "%.1f°", startHeadingDegrees);
            telemetry.addData("AprilTag", "Not found yet...");
            telemetry.update();
            
            sleep(50);
        }
        
        stopDrive();
        return false;
    }
    
    /**
     * Update telemetry display with current status
     */
    protected void updateStatusDisplay() {
        telemetry.addData("🤖 Status", currentStatus);
        telemetry.addData("📍 Position", "x=%.1f\", y=%.1f\", h=%.1f°", 
                          x, y, Math.toDegrees(heading));
        
        // AprilTag status
        if (detectAprilTag()) {
            telemetry.addData("🏷️ AprilTag", "✓ ID %d @ %.1f\"", 
                              desiredTag.id, desiredTag.ftcPose.range);
        } else {
            telemetry.addData("🏷️ AprilTag", "✗ Not Detected");
        }
        
        telemetry.update();
    }
}

