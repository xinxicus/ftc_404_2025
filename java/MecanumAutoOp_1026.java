// Autonomous OpMode based on TeleOp test 10/24
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name="Autonomous 10/26", group="Autonomous")
public class MecanumAutoOp_1026 extends LinearOpMode {
    // ---- Constants for odometry ----
    static final double WHEEL_RADIUS_IN = 1.8898; // 48mm
    static final double GEAR_RATIO = 1.0;
    static final int    TICKS_PER_REV = 537;
    static final double INCHES_PER_TICK =
            (2 * Math.PI * WHEEL_RADIUS_IN * GEAR_RATIO) / TICKS_PER_REV;
    static double LATERAL_MULTIPLIER = 1.05;

    // ---- Drive control constants ----
    private static final double DRIVE_SPEED = 0.6;      // Normal driving speed
    private static final double TURN_SPEED = 0.4;       // Turning speed
    private static final double STRAFE_SPEED = 0.5;     // Strafing speed
    private static final double POSITION_TOLERANCE = 2.0;  // inches
    private static final double HEADING_TOLERANCE = 2.0;   // degrees
    
    // ---- Control gains (tune these if robot is too slow or too aggressive) ----
    private static final double POSITION_GAIN = 0.1;    // Increase if too slow (try 0.15, 0.2), decrease if overshoots
    private static final double HEADING_GAIN = 0.5;     // Decrease if turns too sharp (try 0.3, 0.4), increase if too slow

    // ---- Mecanum drive motors ----
    private DcMotorEx fl, fr, bl, br;
    private IMU imu;
    
    // ---- Accessory motors ----
    private DcMotorEx shootMotor;
    private DcMotor intakeMotor;
    private DcMotor leftIndexMotor;
    private DcMotor rightIndexMotor;
    
    // ---- Shooter constants ----
    private static final double SHOOTER_VELOCITY = 1080;  // Target shooter velocity (ticks/sec)
    private static final double VELOCITY_TOLERANCE = 50;  // Velocity tolerance
    private static final double INDEXER_PULSE_MS = 300;   // How long to pulse each indexer (milliseconds)
    private static final double INDEXER_SEQUENCE_DELAY_MS = 200;  // Delay between left and right indexer (milliseconds) - ADJUST AS NEEDED
    
    // ---- Pose tracking ----
    private double x = 0, y = 0, heading = 0;
    private double lastImuYaw = 0;
    private int lastFL, lastFR, lastBL, lastBR;

    // ---- AprilTag variables ----
    final double DESIRED_DISTANCE = 55.0; 
    final double SPEED_GAIN = 0.035;   
    final double TURN_GAIN = 0.017;   
    final double MAX_AUTO_SPEED = 0.75;   
    final double MAX_AUTO_TURN = 0.25;
    final double BEARING_OFFSET = 14.0;  // Camera angle offset
    final double YAW_OFFSET = 11.0;       // Camera lateral offset
    private static final boolean USE_WEBCAM = true;  
    private static final int DESIRED_TAG_ID = 20;    
    private VisionPortal visionPortal;               
    private AprilTagProcessor aprilTag;              
    private AprilTagDetection desiredTag = null;
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // ---- Initialize hardware ----
        initializeDriveMotors();
        initializeIMU();
        initializeAccessoryMotors();
        initAprilTag();
        
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        // Reset encoders and pose
        resetPose();

        telemetry.addLine("Autonomous Ready. Press ▶ to start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // ================================================
            // YOUR AUTONOMOUS ROUTINE HERE
            // ================================================
            
            // Example autonomous sequence:
            exampleAutonomousRoutine();
            
            // ================================================
            // END OF AUTONOMOUS ROUTINE
            // ================================================
            
            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.update();
        }
    }
    
    /**
     * Example autonomous routine - replace with your actual routine
     */
    private void exampleAutonomousRoutine() {
        telemetry.addLine("Starting autonomous routine...");
        telemetry.update();
        
        // Example 1: Drive forward 24 inches from starting position
        driveToPosition(24, 0, 0, 5.0);
        
        // Example 2: Strafe right 12 inches (now at x=24, y=12)
        driveToPosition(24, 12, 0, 5.0);
        
        // Example 3: Turn to face 90 degrees (right)
        turnToHeading(90, 3.0);
        
        // Example 4: Spin up shooter and shoot 3 times
        if (spinUpShooter(2.0)) {  // Wait up to 2 seconds to reach speed
            // Option A: Use default timing (300ms pulse, 200ms between indexers)
            shoot(3, 0.5);  // Shoot 3 times with 0.5s between shots
            
            // Option B: Use custom timing if default doesn't work well
            // shoot(3, 0.5, 350, 250);  // 350ms pulse, 250ms between indexers
        }
        stopShooter();
        
        // Example 5: Run intake for 2 seconds to collect game pieces
        runIntake(2.0);
        
        // Example 6: Drive to AprilTag if visible (useful for precise positioning)
        boolean reachedTag = driveToAprilTag(5.0);
        if (reachedTag) {
            telemetry.addLine("Successfully aligned with AprilTag!");
            telemetry.update();
            sleep(500);
        }
        
        // Example 7: Drive back to starting position
        driveToPosition(0, 0, 0, 5.0);
    }

    // ================================================================
    // HARDWARE INITIALIZATION METHODS
    // ================================================================
    
    private void initializeDriveMotors() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Right side reversed
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Open-loop control
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    private void initializeIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }
    
    private void initializeAccessoryMotors() {
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftIndexMotor = hardwareMap.dcMotor.get("leftIndexMotor");
        rightIndexMotor = hardwareMap.dcMotor.get("rightIndexMotor");
        
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIndexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIndexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ================================================================
    // AUTONOMOUS DRIVE METHODS
    // ================================================================
    
    /**
     * Drive to a target position (field-centric)
     * @param targetX target X position in inches
     * @param targetY target Y position in inches
     * @param targetHeading target heading in degrees
     * @param timeout maximum time in seconds
     */
    public void driveToPosition(double targetX, double targetY, double targetHeading, double timeout) {
        runtime.reset();
        
        while (opModeIsActive() && runtime.seconds() < timeout) {
            updatePose();
            
            // Calculate errors
            double errorX = targetX - x;
            double errorY = targetY - y;
            double errorHeading = normalizeAngle(Math.toRadians(targetHeading) - heading);
            
            // Check if we've reached the target
            double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);
            if (distanceError < POSITION_TOLERANCE && Math.abs(Math.toDegrees(errorHeading)) < HEADING_TOLERANCE) {
                break;
            }
            
            // Convert field-relative errors to robot-relative
            double robotErrorX = errorX * Math.cos(-heading) - errorY * Math.sin(-heading);
            double robotErrorY = errorX * Math.sin(-heading) + errorY * Math.cos(-heading);
            
            // Calculate drive powers using proportional control
            double fwd = Range.clip(robotErrorX * POSITION_GAIN, -DRIVE_SPEED, DRIVE_SPEED);
            double str = Range.clip(robotErrorY * POSITION_GAIN, -STRAFE_SPEED, STRAFE_SPEED);
            double yaw = Range.clip(errorHeading * HEADING_GAIN, -TURN_SPEED, TURN_SPEED);
            
            // Apply mecanum drive
            setDrivePower(fwd, str, yaw);
            
            telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f°", targetX, targetY, targetHeading);
            telemetry.addData("Current", "X: %.1f, Y: %.1f, H: %.1f°", x, y, Math.toDegrees(heading));
            telemetry.addData("Error", "Dist: %.1f in, Heading: %.1f°", distanceError, Math.toDegrees(errorHeading));
            telemetry.update();
        }
        
        stopDriving();
    }
    
    /**
     * Drive forward/backward for a specific distance
     * @param inches distance to drive (positive = forward, negative = backward)
     * @param timeout maximum time in seconds
     */
    public void driveDistance(double inches, double timeout) {
        double targetX = x + inches * Math.cos(heading);
        double targetY = y + inches * Math.sin(heading);
        driveToPosition(targetX, targetY, Math.toDegrees(heading), timeout);
    }
    
    /**
     * Strafe left/right for a specific distance
     * @param inches distance to strafe (positive = right, negative = left)
     * @param timeout maximum time in seconds
     */
    public void strafeDistance(double inches, double timeout) {
        double targetX = x + inches * Math.cos(heading + Math.PI/2);
        double targetY = y + inches * Math.sin(heading + Math.PI/2);
        driveToPosition(targetX, targetY, Math.toDegrees(heading), timeout);
    }
    
    /**
     * Turn to a specific heading
     * @param targetDegrees target heading in degrees
     * @param timeout maximum time in seconds
     */
    public void turnToHeading(double targetDegrees, double timeout) {
        runtime.reset();
        double targetRad = Math.toRadians(targetDegrees);
        
        while (opModeIsActive() && runtime.seconds() < timeout) {
            updatePose();
            
            double errorHeading = normalizeAngle(targetRad - heading);
            
            if (Math.abs(Math.toDegrees(errorHeading)) < HEADING_TOLERANCE) {
                break;
            }
            
            double yaw = Range.clip(errorHeading * HEADING_GAIN, -TURN_SPEED, TURN_SPEED);
            setDrivePower(0, 0, yaw);
            
            telemetry.addData("Target Heading", "%.1f°", targetDegrees);
            telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(heading));
            telemetry.addData("Error", "%.1f°", Math.toDegrees(errorHeading));
            telemetry.update();
        }
        
        stopDriving();
    }
    
    /**
     * Drive to AprilTag
     * @param timeout maximum time in seconds
     * @return true if successfully reached target, false if timeout or target lost
     */
    public boolean driveToAprilTag(double timeout) {
        runtime.reset();
        
        while (opModeIsActive() && runtime.seconds() < timeout) {
            updatePose();
            
            boolean targetFound = detectAprilTag();
            
            if (!targetFound) {
                telemetry.addData("AprilTag", "Target not found!");
                telemetry.update();
                stopDriving();
                return false;
            }
            
            // Calculate drive commands based on AprilTag position
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double yawError = (desiredTag.ftcPose.yaw - YAW_OFFSET);
            double headingError = (desiredTag.ftcPose.bearing - BEARING_OFFSET);
            
            // Check if we've reached the target
            if (Math.abs(rangeError) < 3.0 && Math.abs(yawError) < 3.0 && Math.abs(headingError) < 3.0) {
                stopDriving();
                telemetry.addData("AprilTag", "Target reached!");
                telemetry.update();
                return true;
            }
            
            double fwd = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double str = Range.clip(yawError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double yaw = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            
            setDrivePower(fwd, str, yaw);
            
            telemetry.addData("AprilTag", "Driving to target...");
            telemetry.addData("Range", "%.1f\" (target: %.1f\")", desiredTag.ftcPose.range, DESIRED_DISTANCE);
            telemetry.addData("Yaw Error", "%.1f°", yawError);
            telemetry.addData("Bearing Error", "%.1f°", headingError);
            telemetry.update();
        }
        
        stopDriving();
        telemetry.addData("AprilTag", "Timeout!");
        telemetry.update();
        return false;
    }

    // ================================================================
    // ACCESSORY CONTROL METHODS
    // ================================================================
    
    /**
     * Spin up shooter to target velocity
     * @param timeout maximum time to wait in seconds
     * @return true if reached target velocity, false if timeout
     */
    public boolean spinUpShooter(double timeout) {
        runtime.reset();
        shootMotor.setPower(1.0);
        
        while (opModeIsActive() && runtime.seconds() < timeout) {
            double velocity = Math.abs(shootMotor.getVelocity());
            
            if (velocity >= SHOOTER_VELOCITY - VELOCITY_TOLERANCE) {
                telemetry.addData("Shooter", "Ready! (%.0f ticks/sec)", velocity);
                telemetry.update();
                return true;
            }
            
            telemetry.addData("Shooter", "Spinning up... %.0f/%.0f ticks/sec", velocity, SHOOTER_VELOCITY);
            telemetry.update();
            sleep(50);
        }
        
        return false;
    }
    
    /**
     * Stop shooter motor
     */
    public void stopShooter() {
        shootMotor.setPower(0);
    }
    
    /**
     * Shoot specified number of times (indexers fire in sequence: left then right)
     * Uses default timing constants
     * @param count number of shots (each shot = left indexer pulse + right indexer pulse)
     * @param delayBetweenShots delay between complete shots in seconds
     */
    public void shoot(int count, double delayBetweenShots) {
        shoot(count, delayBetweenShots, INDEXER_PULSE_MS, INDEXER_SEQUENCE_DELAY_MS);
    }
    
    /**
     * Shoot specified number of times with custom timing (indexers fire in sequence: left then right)
     * @param count number of shots (each shot = left indexer pulse + right indexer pulse)
     * @param delayBetweenShots delay between complete shots in seconds
     * @param indexerPulseMs how long to pulse each indexer in milliseconds
     * @param sequenceDelayMs delay between left and right indexer in milliseconds
     */
    public void shoot(int count, double delayBetweenShots, double indexerPulseMs, double sequenceDelayMs) {
        for (int i = 0; i < count && opModeIsActive(); i++) {
            telemetry.addData("Shooting", "%d/%d (L→R sequence)", i + 1, count);
            telemetry.update();
            
            // Fire LEFT indexer first
            leftIndexMotor.setPower(1.0);
            sleep((long)indexerPulseMs);
            leftIndexMotor.setPower(0);
            
            // Delay between left and right indexers
            sleep((long)sequenceDelayMs);
            
            // Fire RIGHT indexer second
            rightIndexMotor.setPower(1.0);
            sleep((long)indexerPulseMs);
            rightIndexMotor.setPower(0);
            
            // Wait before next shot (if not the last shot)
            if (i < count - 1) {
                sleep((long)(delayBetweenShots * 1000));
            }
        }
    }
    
    /**
     * Run intake for specified duration
     * @param seconds duration in seconds
     */
    public void runIntake(double seconds) {
        intakeMotor.setPower(1.0);
        sleep((long)(seconds * 1000));
        intakeMotor.setPower(0);
    }
    
    /**
     * Run indexers in sequence for specified duration
     * (Alternates left-right-left-right pattern)
     * @param seconds total duration in seconds
     */
    public void runIndexers(double seconds) {
        long endTime = System.currentTimeMillis() + (long)(seconds * 1000);
        
        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            // Fire LEFT indexer
            leftIndexMotor.setPower(1.0);
            sleep((long)INDEXER_PULSE_MS);
            leftIndexMotor.setPower(0);
            
            if (System.currentTimeMillis() >= endTime) break;
            
            // Delay between indexers
            sleep((long)INDEXER_SEQUENCE_DELAY_MS);
            
            if (System.currentTimeMillis() >= endTime) break;
            
            // Fire RIGHT indexer
            rightIndexMotor.setPower(1.0);
            sleep((long)INDEXER_PULSE_MS);
            rightIndexMotor.setPower(0);
            
            if (System.currentTimeMillis() >= endTime) break;
            
            // Brief pause before next cycle
            sleep((long)INDEXER_SEQUENCE_DELAY_MS);
        }
        
        // Ensure both are off
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
    }

    // ================================================================
    // UTILITY METHODS
    // ================================================================
    
    /**
     * Set drive power using mecanum kinematics
     * @param fwd forward power (-1 to 1)
     * @param str strafe power (-1 to 1)
     * @param yaw rotation power (-1 to 1)
     */
    private void setDrivePower(double fwd, double str, double yaw) {
        double flPow = fwd + str + yaw;
        double blPow = fwd - str + yaw;
        double frPow = fwd - str - yaw;
        double brPow = fwd + str - yaw;

        // Normalize to [-1, 1]
        double max = Math.max(1.0, Math.max(Math.abs(flPow),
                    Math.max(Math.abs(blPow), Math.abs(frPow))));
        max = Math.max(max, Math.abs(brPow));
        flPow /= max;
        blPow /= max;
        frPow /= max;
        brPow /= max;

        fl.setPower(flPow);
        bl.setPower(blPow);
        fr.setPower(frPow);
        br.setPower(brPow);
    }
    
    /**
     * Stop all drive motors
     */
    private void stopDriving() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    
    /**
     * Update robot pose using encoders and IMU
     */
    private void updatePose() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double imuYaw = ypr.getYaw(AngleUnit.RADIANS);
        double dTheta = normalizeAngle(imuYaw - lastImuYaw);
        lastImuYaw = imuYaw;

        int cFL = fl.getCurrentPosition();
        int cFR = fr.getCurrentPosition();
        int cBL = bl.getCurrentPosition();
        int cBR = br.getCurrentPosition();
        
        double dFL = (cFL - lastFL) * INCHES_PER_TICK;
        double dFR = (cFR - lastFR) * INCHES_PER_TICK;
        double dBL = (cBL - lastBL) * INCHES_PER_TICK;
        double dBR = (cBR - lastBR) * INCHES_PER_TICK;
        
        lastFL = cFL;
        lastFR = cFR;
        lastBL = cBL;
        lastBR = cBR;

        double dxR = (dFL + dFR + dBL + dBR) / 4.0;
        double dyR = (-dFL + dFR + dBL - dBR) / 4.0 * LATERAL_MULTIPLIER;

        double hMid = heading + dTheta / 2.0;
        double c = Math.cos(hMid);
        double s = Math.sin(hMid);
        
        x += dxR * c - dyR * s;
        y += dxR * s + dyR * c;
        heading = normalizeAngle(heading + dTheta);
    }
    
    /**
     * Reset pose to origin
     */
    private void resetPose() {
        x = 0;
        y = 0;
        heading = 0;
        
        // Read current IMU yaw after reset (should be ~0)
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        lastImuYaw = ypr.getYaw(AngleUnit.RADIANS);
        
        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();
    }
    
    /**
     * Normalize angle to [-PI, PI]
     */
    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    // ================================================================
    // APRILTAG METHODS
    // ================================================================
    
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
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
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    
    private boolean detectAprilTag() {
        desiredTag = null;
        
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    desiredTag = detection;
                    return true;
                }
            }
        }
        
        return false;
    }
}

