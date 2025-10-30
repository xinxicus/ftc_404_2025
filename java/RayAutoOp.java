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

/**
 * Autonomous OpMode based on RayTest2 Button X sequence
 * 
 * Sequence:
 * 1. Drive forward 10 inches
 * 2. Scan for AprilTag and drive to it
 * 3. Shoot for 8 seconds
 * 4. Reset pose
 * 5. Drive backward to loading zone
 * 6. Reverse mode (clear jams)
 * 7. Drive to AprilTag again
 * 8. Shoot for 8 seconds again
 */
@Autonomous(name="Ray Auto Op")
public class RayAutoOp extends LinearOpMode {
    // ---- Odometry constants ----
    static final double WHEEL_RADIUS_IN = 1.8898;
    static final double GEAR_RATIO = 1.0;
    static final int    TICKS_PER_REV = 537;
    static final double INCHES_PER_TICK =
            (2 * Math.PI * WHEEL_RADIUS_IN * GEAR_RATIO) / TICKS_PER_REV;
    static double LATERAL_MULTIPLIER = 1.05;

    // ---- Autonomous driving constants ----
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.8;
    static final double HEADING_THRESHOLD = 2.0;
    static final double DISTANCE_THRESHOLD = 2.0;
    
    // ---- Drive motors ----
    private DcMotorEx fl, fr, bl, br;
    private IMU imu;

    // ---- Accessory motors ----
    private DcMotorEx shootMotor;
    private DcMotor intakeMotor;
    private DcMotor leftIndexMotor;
    private DcMotor rightIndexMotor;
    
    // ---- Shooter constants ----
    private static final double SHOOTER_VELOCITY = 1200;
    private static final double INDEXER_ACTIVATION_VELOCITY = 1175;
    private static final double MAX_REVERSE_VELOCITY = 400;
    
    // ---- Pose tracking ----
    private double x = 0, y = 0, heading = 0;
    private double lastImuYaw = 0;
    private int lastFL, lastFR, lastBL, lastBR;
    
    // ---- AprilTag variables ----
    final double DESIRED_DISTANCE = 65.0;
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
    
    // ---- Sequence tracking ----
    private String currentStep = "Initializing";

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        
        // Initialize AprilTag vision
        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }
        
        telemetry.addLine("=== RAY AUTONOMOUS MODE ===");
        telemetry.addLine();
        telemetry.addLine("Sequence:");
        telemetry.addLine("1. Drive forward 10\"");
        telemetry.addLine("2. Scan & drive to AprilTag");
        telemetry.addLine("3. Shoot for 8 seconds");
        telemetry.addLine("4. Reset pose");
        telemetry.addLine("5. Drive back to loading zone");
        telemetry.addLine("6. Clear jams (reverse mode)");
        telemetry.addLine("7. Drive to AprilTag again");
        telemetry.addLine("8. Shoot for 8 seconds");
        telemetry.addLine();
        telemetry.addLine("Press ▶ to start");
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) return;

        // ==================== AUTONOMOUS SEQUENCE ====================
        
        try {
            // Step 1: Move forward 10 inches
            currentStep = "Step 1: Drive Forward 10\"";
            updateTelemetry();
            driveDistance(10, 0, DRIVE_SPEED, 5.0);
            sleep(100);
            
            // Step 2: Scan for AprilTag and drive to it
            currentStep = "Step 2: Scanning for AprilTag";
            updateTelemetry();
            if (scanForAprilTag(10.0)) {
                currentStep = "Step 2: Tag Found - Driving";
                updateTelemetry();
                driveToAprilTag(10.0);
                currentStep = "Step 2: Complete";
            } else {
                currentStep = "Step 2: No AprilTag Found";
            }
            updateTelemetry();
            sleep(100);
            
            // Step 3: Shoot for 8 seconds
            currentStep = "Step 3: Shooting (8 sec)";
            updateTelemetry();
            shootSequence(8.0);
            sleep(100);
            
            // Step 4: Reset pose
            currentStep = "Step 4: Resetting Pose";
            updateTelemetry();
            x = 0;
            y = 0;
            heading = 0;
            imu.resetYaw();
            lastImuYaw = 0;
            sleep(1000);
            
            // Step 5: Drive backward to loading zone
            currentStep = "Step 5: Drive to Loading Zone";
            updateTelemetry();
            driveDistance(-76, -12, DRIVE_SPEED, 5.0);
            sleep(100);
            
            // Step 6: Reverse mode (clear jams)
            currentStep = "Step 6: Clearing Jams";
            updateTelemetry();
            reverseMode(10.0);
            sleep(100);
            
            // Step 7: Drive to AprilTag again
            currentStep = "Step 7: Drive to AprilTag";
            updateTelemetry();
            if (detectAprilTag()) {
                driveToAprilTag(10.0);
                currentStep = "Step 7: Complete";
            } else {
                currentStep = "Step 7: No AprilTag Found";
            }
            updateTelemetry();
            sleep(100);
            
            // Step 8: Shoot for 8 seconds again
            currentStep = "Step 8: Shooting (8 sec)";
            updateTelemetry();
            shootSequence(8.0);
            sleep(100);
            
            // Sequence complete
            currentStep = "✓ AUTONOMOUS COMPLETE";
            updateTelemetry();
            
        } catch (Exception e) {
            telemetry.addLine("❌ ERROR: " + e.getMessage());
            telemetry.update();
        }
        
        // Stop all motors
        stopDrive();
        shootMotor.setPower(0);
        intakeMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
    }
    
    // ==================== Hardware Initialization ====================
    
    private void initHardware() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();
        
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftIndexMotor = hardwareMap.dcMotor.get("leftIndexMotor");
        rightIndexMotor = hardwareMap.dcMotor.get("rightIndexMotor");
        
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIndexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIndexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();
    }
    
    // ==================== Pose Tracking ====================
    
    private void updatePose() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double imuYaw = -ypr.getYaw(AngleUnit.RADIANS);
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
    
    // ==================== Autonomous Movement Functions ====================
    
    private void driveDistance(double forwardInches, double strafeInches, double speed, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        double startX = x, startY = y;
        double targetX = startX + forwardInches * Math.cos(heading) - strafeInches * Math.sin(heading);
        double targetY = startY + forwardInches * Math.sin(heading) + strafeInches * Math.cos(heading);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            
            double errorX = targetX - x;
            double errorY = targetY - y;
            double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);
            
            if (distanceError < DISTANCE_THRESHOLD) {
                break;
            }
            
            double c = Math.cos(heading), s = Math.sin(heading);
            double errorForward = errorX * c + errorY * s;
            double errorStrafe = -errorX * s + errorY * c;
            
            double fwd = Math.max(-speed, Math.min(speed, errorForward * 0.1));
            double str = Math.max(-speed, Math.min(speed, errorStrafe * 0.1));
            
            if (Math.abs(fwd) < 0.08 && Math.abs(str) < 0.08) {
                break;
            }
            
            setDrivePower(fwd, str, 0);
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addData("🎯 Target", "x=%.1f, y=%.1f", targetX, targetY);
            telemetry.addData("📍 Current", "x=%.1f, y=%.1f", x, y);
            telemetry.addData("📏 Error", "%.1f inches", distanceError);
            telemetry.update();
        }
        
        stopDrive();
    }
    
    private void turnToHeading(double targetHeadingDegrees, double speed, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);
        targetHeadingRad = wrap(targetHeadingRad);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            
            double headingError = wrap(targetHeadingRad - heading);
            double headingErrorDegrees = Math.toDegrees(headingError);
            
            if (Math.abs(headingErrorDegrees) < HEADING_THRESHOLD) {
                break;
            }
            
            double turnPower = headingError * 0.6;
            turnPower = Math.max(-speed, Math.min(speed, turnPower));
            
            if (Math.abs(turnPower) < 0.15) {
                turnPower = Math.signum(turnPower) * 0.15;
            }
            
            if (Math.abs(headingErrorDegrees) < HEADING_THRESHOLD * 0.5) {
                break;
            }
            
            setDrivePower(0, 0, turnPower);
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addData("🎯 Target Heading", "%.1f°", Math.toDegrees(targetHeadingRad));
            telemetry.addData("🧭 Current Heading", "%.1f°", Math.toDegrees(heading));
            telemetry.addData("↻ Error", "%.1f°", headingErrorDegrees);
            telemetry.update();
        }
        
        stopDrive();
    }
    
    private void driveToAprilTag(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            
            boolean targetFound = detectAprilTag();
            
            if (!targetFound) {
                telemetry.addData("Current Step", currentStep);
                telemetry.addLine("⚠️ Target lost!");
                telemetry.update();
                break;
            }
            
            double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double yawError = desiredTag.ftcPose.yaw - YAW_OFFSET;
            double headingError = desiredTag.ftcPose.bearing - BEARING_OFFSET;
            
            if (Math.abs(rangeError) < 2.0 && Math.abs(yawError) < 3.0 && Math.abs(headingError) < 3.0) {
                telemetry.addData("Current Step", currentStep);
                telemetry.addLine("✓ Reached target!");
                telemetry.update();
                break;
            }
            
            double fwd = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, rangeError * SPEED_GAIN));
            double str = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, yawError * SPEED_GAIN));
            double yaw = -Math.max(-MAX_AUTO_TURN, Math.min(MAX_AUTO_TURN, headingError * TURN_GAIN));
            
            setDrivePower(fwd, str, yaw);
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addData("🏷️ AprilTag", "ID %d", desiredTag.id);
            telemetry.addData("📏 Range", "%.1f\" → %.1f\"", desiredTag.ftcPose.range, DESIRED_DISTANCE);
            telemetry.addData("↔️ Yaw", "%.1f°", desiredTag.ftcPose.yaw);
            telemetry.update();
        }
        
        stopDrive();
    }
    
    private void shootSequence(double durationSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        final double INDEXER_ACTIVE_TIME = 1.0;
        final double DELAY_BETWEEN_SHOTS = 1.0;
        final int LOADBALLS_BEFORE_SHOT_DURATION = 1000;
        
        shootMotor.setPower(1.0);
        
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("🎯 Spinning up shooter...");
        telemetry.update();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 3.0) {
            double velocity = Math.abs(shootMotor.getVelocity());
            controlShooterVelocity(velocity, 1.0);
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addData("🎯 Shooter", "%.0f / %.0f ticks/sec", velocity, INDEXER_ACTIVATION_VELOCITY);
            telemetry.update();
            
            if (velocity >= INDEXER_ACTIVATION_VELOCITY) {
                break;
            }
            sleep(50);
        }
        
        String[] sequence = {"RIGHT", "LEFT", "RIGHT", "LEFT"};
        
        for (int shot = 0; shot < 4 && opModeIsActive(); shot++) {
            String currentIndexer = sequence[shot];
            
            if (shot == 2) {
                telemetry.addData("Current Step", currentStep);
                telemetry.addLine(String.format("🔄 Loading balls before shot %d...", shot + 1));
                telemetry.update();
                intakeMotor.setPower(1.0);
                sleep(LOADBALLS_BEFORE_SHOT_DURATION);
            }
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addLine(String.format("🚀 Shot %d/4: %s INDEXER", shot + 1, currentIndexer));
            telemetry.update();
            
            if (currentIndexer.equals("RIGHT")) {
                rightIndexMotor.setPower(1.0);
            } else {
                leftIndexMotor.setPower(1.0);
            }
            
            timer.reset();
            while (opModeIsActive() && timer.seconds() < INDEXER_ACTIVE_TIME) {
                double velocity = Math.abs(shootMotor.getVelocity());
                controlShooterVelocity(velocity, 1.0);
                
                telemetry.addData("Current Step", currentStep);
                telemetry.addData("🚀 Shot", "%d/4", shot + 1);
                telemetry.addData("⏱️ Time", "%.2f / %.2f sec", timer.seconds(), INDEXER_ACTIVE_TIME);
                telemetry.update();
                sleep(50);
            }
            
            leftIndexMotor.setPower(0);
            rightIndexMotor.setPower(0);
            
            if (shot < 3) {
                timer.reset();
                while (opModeIsActive() && timer.seconds() < DELAY_BETWEEN_SHOTS) {
                    double velocity = Math.abs(shootMotor.getVelocity());
                    controlShooterVelocity(velocity, 1.0);
                    sleep(50);
                }
            }
        }
        
        shootMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
        intakeMotor.setPower(0);
        
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("✓ Shooting complete!");
        telemetry.update();
        sleep(500);
    }
    
    private void controlShooterVelocity(double currentVelocity, double requestedPower) {
        double adjustedPower = requestedPower;
        
        if (currentVelocity >= SHOOTER_VELOCITY) {
            adjustedPower = -0.1;
        } else if (currentVelocity >= SHOOTER_VELOCITY * 0.95) {
            double velocityRatio = (SHOOTER_VELOCITY - currentVelocity) / (SHOOTER_VELOCITY * 0.05);
            adjustedPower = requestedPower * velocityRatio;
        } else if (currentVelocity >= SHOOTER_VELOCITY * 0.90) {
            adjustedPower = Math.min(requestedPower, 0.5);
        }
        
        shootMotor.setPower(adjustedPower);
    }
    
    // ==================== Drive Control ====================
    
    private void setDrivePower(double fwd, double str, double yaw) {
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
    
    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    
    private static double wrap(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }
    
    // ==================== AprilTag Functions ====================
    
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
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
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
    
    private boolean scanForAprilTag(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        final double SCAN_SPEED = 0.15;
        final double SCAN_ANGLE = 45.0;
        
        double startHeadingDegrees = Math.toDegrees(heading);
        
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("🔍 Scanning LEFT...");
        telemetry.update();
        
        double targetLeft = startHeadingDegrees - SCAN_ANGLE;
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds / 2) {
            updatePose();
            
            if (detectAprilTag()) {
                stopDrive();
                telemetry.addData("Current Step", currentStep);
                telemetry.addLine("✓ AprilTag FOUND!");
                telemetry.update();
                sleep(300);
                return true;
            }
            
            double currentHeadingDegrees = Math.toDegrees(heading);
            double headingError = wrap(Math.toRadians(targetLeft - currentHeadingDegrees));
            
            if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                break;
            }
            
            setDrivePower(0, 0, -SCAN_SPEED);
            sleep(50);
        }
        
        stopDrive();
        sleep(200);
        
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("🔍 Scanning RIGHT...");
        telemetry.update();
        
        double targetRight = startHeadingDegrees + SCAN_ANGLE;
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds / 2) {
            updatePose();
            
            if (detectAprilTag()) {
                stopDrive();
                telemetry.addData("Current Step", currentStep);
                telemetry.addLine("✓ AprilTag FOUND!");
                telemetry.update();
                sleep(300);
                return true;
            }
            
            double currentHeadingDegrees = Math.toDegrees(heading);
            double headingError = wrap(Math.toRadians(targetRight - currentHeadingDegrees));
            
            if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                break;
            }
            
            setDrivePower(0, 0, SCAN_SPEED);
            sleep(50);
        }
        
        stopDrive();
        
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("❌ No AprilTag found - returning...");
        telemetry.update();
        turnToHeading(startHeadingDegrees, TURN_SPEED, 3.0);
        
        return false;
    }

    private void loadBalls(double load_duration){
        ElapsedTime timer = new ElapsedTime();
        
        intakeMotor.setPower(1.0);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < load_duration) {
            updatePose();
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addData("Status", "🔄 LOADING BALLS");
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), load_duration);
            telemetry.update();
            
            sleep(50);
        }
        
        intakeMotor.setPower(0);
        sleep(500);
    }
    
    private void reverseMode(double duration) {
        ElapsedTime timer = new ElapsedTime();
        
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("⚠️ REVERSE MODE - Clearing jams...");
        telemetry.update();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < duration) {
            updatePose();
            
            double currentShooterVelocity = Math.abs(shootMotor.getVelocity());
            
            double reversePower = -1.0;
            if (currentShooterVelocity >= MAX_REVERSE_VELOCITY) {
                reversePower = 0;
            } else if (currentShooterVelocity >= MAX_REVERSE_VELOCITY * 0.95) {
                double velocityRatio = (MAX_REVERSE_VELOCITY - currentShooterVelocity) / (MAX_REVERSE_VELOCITY * 0.05);
                reversePower = -1.0 * velocityRatio;
            } else if (currentShooterVelocity >= MAX_REVERSE_VELOCITY * 0.90) {
                reversePower = -0.5;
            }
            
            shootMotor.setPower(reversePower);
            leftIndexMotor.setPower(-1.0);
            rightIndexMotor.setPower(-1.0);
            intakeMotor.setPower(0.0);
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addData("Status", "⚠️ CLEARING JAMS");
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), duration);
            telemetry.update();
            
            sleep(50);
        }
        
        shootMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
        intakeMotor.setPower(0);
        
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("✓ Reverse mode complete!");
        telemetry.update();
        sleep(500);
    }
    
    private void updateTelemetry() {
        telemetry.clear();
        telemetry.addData("🤖 Current Step", currentStep);
        telemetry.addData("📍 Position", "x=%.1f\", y=%.1f\", h=%.1f°", x, y, Math.toDegrees(heading));
        telemetry.update();
    }
}

