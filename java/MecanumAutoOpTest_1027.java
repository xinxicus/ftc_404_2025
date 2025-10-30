// Autonomous OpMode based on DriveTestTeleOp_1027 hardware configuration
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="AutoOp Test 10/27")
public class MecanumAutoOpTest_1027 extends LinearOpMode {
    // ---- Odometry constants (from TeleOp) ----
    static final double WHEEL_RADIUS_IN = 1.8898; // 48mm
    static final double GEAR_RATIO = 1.0;
    static final int    TICKS_PER_REV = 537;
    static final double INCHES_PER_TICK =
            (2 * Math.PI * WHEEL_RADIUS_IN * GEAR_RATIO) / TICKS_PER_REV;
    static double LATERAL_MULTIPLIER = 1.05;

    // ---- Autonomous driving constants ----
    static final double DRIVE_SPEED = 0.6;  // Normal driving speed
    static final double TURN_SPEED = 0.4;   // Turning speed
    static final double HEADING_THRESHOLD = 2.0;  // Degrees
    static final double DISTANCE_THRESHOLD = 1.0; // Inches
    
    // ---- Drive motors ----
    private DcMotorEx fl, fr, bl, br;
    private IMU imu;
    
    // ---- Accessory motors ----
    private DcMotorEx shootMotor;
    private DcMotor intakeMotor;
    private DcMotor leftIndexMotor;
    private DcMotor rightIndexMotor;
    
    // ---- Shooter constants (from TeleOp) ----
    private static final double SHOOTER_VELOCITY = 1100;  // Maximum shooter velocity (ticks/sec)
    private static final double INDEXER_ACTIVATION_VELOCITY = 1075;  // Velocity threshold for indexers
    
    // ---- Pose tracking ----
    private double x = 0, y = 0, heading = 0;
    private double lastImuYaw = 0;
    private int lastFL, lastFR, lastBL, lastBR;
    
    // ---- AprilTag variables (from TeleOp) ----
    final double DESIRED_DISTANCE = 55.0;
    final double SPEED_GAIN = 0.035;
    final double TURN_GAIN = 0.017;
    final double MAX_AUTO_SPEED = 0.55;
    final double MAX_AUTO_TURN = 0.25;
    final double BEARING_OFFSET = 0.0;
    final double YAW_OFFSET = 0.0;
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 20;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    
    // ---- Timing ----
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        
        // Initialize AprilTag vision
        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }
        
        // Wait for start
        telemetry.addLine("Autonomous Ready. Press ▶ to start");
        telemetry.addData("Starting Position", "x=%.1f, y=%.1f, h=%.1f°", x, y, Math.toDegrees(heading));
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            // ===== AUTONOMOUS ROUTINE STARTS HERE =====
            
            // Example autonomous sequence:
            telemetry.addLine("Starting autonomous sequence...");
            telemetry.update();
            
            // 1. Drive forward 24 inches
            telemetry.addLine("Step 1: Drive forward 24 inches");
            telemetry.update();
            driveDistance(24, 0, DRIVE_SPEED, 5.0);
            sleep(500);
            
            // 2. Strafe right 12 inches
            telemetry.addLine("Step 2: Strafe right 12 inches");
            telemetry.update();
            driveDistance(0, 12, DRIVE_SPEED, 5.0);
            sleep(500);
            
            // 3. Turn 90 degrees
            telemetry.addLine("Step 3: Turn 90 degrees");
            telemetry.update();
            turnToHeading(90, TURN_SPEED, 5.0);
            sleep(500);
            
            // 4. Optional: Drive to AprilTag if detected
            telemetry.addLine("Step 4: Looking for AprilTag...");
            telemetry.update();
            if (detectAprilTag()) {
                telemetry.addLine("AprilTag found! Driving to target...");
                telemetry.update();
                driveToAprilTag(10.0);
            } else {
                telemetry.addLine("No AprilTag detected, skipping...");
                telemetry.update();
            }
            
            // 5. Example: Activate shooter and score
            telemetry.addLine("Step 5: Activating shooter...");
            telemetry.update();
            shootSequence(2.0);  // Shoot for 2 seconds
            
            telemetry.addLine("✓ Autonomous sequence complete!");
            telemetry.addData("Final Position", "x=%.1f, y=%.1f, h=%.1f°", x, y, Math.toDegrees(heading));
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
    }
    
    /**
     * Initialize all hardware components
     */
    private void initHardware() {
        // Map drive motors
        fl = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Set motor directions (right side reversed)
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Configure motors for open-loop control
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        
        // Initialize accessory motors
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftIndexMotor = hardwareMap.dcMotor.get("leftIndexMotor");
        rightIndexMotor = hardwareMap.dcMotor.get("rightIndexMotor");
        
        // Set accessory motor directions
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIndexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIndexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Configure accessory motors
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize encoder positions
        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();
    }
    
    /**
     * Update robot pose based on encoder odometry and IMU
     */
    private void updatePose() {
        // Read IMU heading
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double imuYaw = ypr.getYaw(AngleUnit.RADIANS);
        double dTheta = wrap(imuYaw - lastImuYaw);
        lastImuYaw = imuYaw;

        // Read encoder changes
        int cFL = fl.getCurrentPosition(), cFR = fr.getCurrentPosition();
        int cBL = bl.getCurrentPosition(), cBR = br.getCurrentPosition();
        double dFL = (cFL - lastFL) * INCHES_PER_TICK;
        double dFR = (cFR - lastFR) * INCHES_PER_TICK;
        double dBL = (cBL - lastBL) * INCHES_PER_TICK;
        double dBR = (cBR - lastBR) * INCHES_PER_TICK;
        lastFL = cFL; lastFR = cFR; lastBL = cBL; lastBR = cBR;

        // Calculate robot-relative displacement
        double dxR = (dFL + dFR + dBL + dBR) / 4.0;
        double dyR = (dFL - dFR - dBL + dBR) / 4.0 * LATERAL_MULTIPLIER;

        // Transform to field coordinates
        double hMid = heading + dTheta / 2.0;
        double c = Math.cos(hMid), s = Math.sin(hMid);
        x += dxR * c - dyR * s;
        y += dxR * s + dyR * c;
        heading = wrap(heading + dTheta);
    }
    
    /**
     * Drive a specific distance in field coordinates
     * @param forwardInches Distance to drive forward (positive = forward)
     * @param strafeInches Distance to strafe right (positive = right)
     * @param speed Speed multiplier (0 to 1)
     * @param timeoutSeconds Maximum time to complete the movement
     */
    private void driveDistance(double forwardInches, double strafeInches, double speed, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        // Calculate target position in field coordinates
        double startX = x, startY = y;
        double targetX = startX + forwardInches * Math.cos(heading) - strafeInches * Math.sin(heading);
        double targetY = startY + forwardInches * Math.sin(heading) + strafeInches * Math.cos(heading);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            
            // Calculate error in field coordinates
            double errorX = targetX - x;
            double errorY = targetY - y;
            double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);
            
            // Check if we've reached the target
            if (distanceError < DISTANCE_THRESHOLD) {
                break;
            }
            
            // Transform error to robot coordinates
            double c = Math.cos(heading), s = Math.sin(heading);
            double errorForward = errorX * c + errorY * s;
            double errorStrafe = -errorX * s + errorY * c;
            
            // Calculate motor powers (proportional control)
            double fwd = Math.max(-speed, Math.min(speed, errorForward * 0.1));
            double str = Math.max(-speed, Math.min(speed, errorStrafe * 0.1));
            
            // Add minimum power threshold to prevent oscillation
            if (Math.abs(fwd) < 0.08 && Math.abs(str) < 0.08) {
                break;
            }
            
            setDrivePower(fwd, str, 0);
            
            telemetry.addData("Target", "x=%.1f, y=%.1f", targetX, targetY);
            telemetry.addData("Current", "x=%.1f, y=%.1f, h=%.1f°", x, y, Math.toDegrees(heading));
            telemetry.addData("Error", "%.1f inches", distanceError);
            telemetry.update();
        }
        
        stopDrive();
    }
    
    /**
     * Turn to a specific heading (field-relative)
     * @param targetDegrees Target heading in degrees (0 = initial heading)
     * @param speed Turning speed (0 to 1)
     * @param timeoutSeconds Maximum time to complete the turn
     */
    private void turnToHeading(double targetDegrees, double speed, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        double targetHeading = Math.toRadians(targetDegrees);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            
            // Calculate heading error
            double headingError = wrap(targetHeading - heading);
            double headingErrorDeg = Math.toDegrees(headingError);
            
            // Check if we've reached the target
            if (Math.abs(headingErrorDeg) < HEADING_THRESHOLD) {
                break;
            }
            
            // Calculate turning power (proportional control)
            double turnPower = Math.max(-speed, Math.min(speed, headingError * 0.5));
            
            // Add minimum power threshold to prevent oscillation
            if (Math.abs(turnPower) < 0.1) {
                break;
            }
            
            setDrivePower(0, 0, turnPower);
            
            telemetry.addData("Target Heading", "%.1f°", targetDegrees);
            telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(heading));
            telemetry.addData("Error", "%.1f°", headingErrorDeg);
            telemetry.update();
        }
        
        stopDrive();
    }
    
    /**
     * Drive to AprilTag using vision feedback
     * @param timeoutSeconds Maximum time to complete the approach
     */
    private void driveToAprilTag(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            
            // Detect AprilTag
            boolean targetFound = detectAprilTag();
            
            if (!targetFound) {
                telemetry.addLine("Target lost!");
                break;
            }
            
            // Calculate approach errors
            double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double yawError = desiredTag.ftcPose.yaw - YAW_OFFSET;
            double headingError = desiredTag.ftcPose.bearing - BEARING_OFFSET;
            
            // Check if we're at the target
            if (Math.abs(rangeError) < 2.0 && Math.abs(yawError) < 3.0 && Math.abs(headingError) < 3.0) {
                telemetry.addLine("✓ Reached AprilTag target!");
                break;
            }
            
            // Calculate control outputs
            double fwd = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, rangeError * SPEED_GAIN));
            double str = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, yawError * SPEED_GAIN));
            double yaw = -Math.max(-MAX_AUTO_TURN, Math.min(MAX_AUTO_TURN, headingError * TURN_GAIN));
            
            setDrivePower(fwd, str, yaw);
            
            telemetry.addData("AprilTag", "ID %d", desiredTag.id);
            telemetry.addData("Range", "%.1f\" (target: %.1f\")", desiredTag.ftcPose.range, DESIRED_DISTANCE);
            telemetry.addData("Yaw", "%.1f°", desiredTag.ftcPose.yaw);
            telemetry.addData("Bearing", "%.1f°", desiredTag.ftcPose.bearing);
            telemetry.update();
        }
        
        stopDrive();
    }
    
    /**
     * Execute shooting sequence
     * @param durationSeconds How long to run the shooter
     */
    private void shootSequence(double durationSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        // Spin up shooter
        shootMotor.setPower(1.0);
        
        timer.reset();
        boolean indexersActivated = false;
        
        while (opModeIsActive() && timer.seconds() < durationSeconds) {
            // Read shooter velocity
            double velocity = Math.abs(shootMotor.getVelocity());
            
            // Activate indexers when shooter reaches target velocity
            if (velocity >= INDEXER_ACTIVATION_VELOCITY && !indexersActivated) {
                leftIndexMotor.setPower(1.0);
                rightIndexMotor.setPower(1.0);
                indexersActivated = true;
                telemetry.addLine("✓ Indexers activated!");
            }
            
            telemetry.addData("Shooter", "%.0f / %.0f ticks/sec", velocity, SHOOTER_VELOCITY);
            telemetry.addData("Indexers", indexersActivated ? "ACTIVE" : "WAITING");
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), durationSeconds);
            telemetry.update();
            
            sleep(50);
        }
        
        // Stop all shooting motors
        shootMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
        
        telemetry.addLine("✓ Shooting sequence complete");
        telemetry.update();
    }
    
    /**
     * Set drive power using mecanum drive calculations
     * @param fwd Forward power (-1 to 1)
     * @param str Strafe power (-1 to 1)
     * @param yaw Rotation power (-1 to 1)
     */
    private void setDrivePower(double fwd, double str, double yaw) {
        // Mecanum drive calculations
        double flPow = fwd + str + yaw;
        double blPow = fwd - str + yaw;
        double frPow = fwd - str - yaw;
        double brPow = fwd + str - yaw;

        // Normalize to [-1, 1]
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
    
    /**
     * Stop all drive motors
     */
    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    
    /**
     * Wrap angle to [-π, π]
     */
    private static double wrap(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }
    
    // ==================== AprilTag Methods (from TeleOp) ====================
    
    /**
     * Initialize the AprilTag processor
     */
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

    /**
     * Manually set camera exposure and gain (for webcams only)
     */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }
        
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
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
     * Detect AprilTag and set desiredTag if found
     * @return true if the desired tag is found, false otherwise
     */
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

