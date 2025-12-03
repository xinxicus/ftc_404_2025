package org.firstinspires.ftc.teamcode.error404;

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

@Autonomous(name = "AutoOp Red close start daniel 12/6")
public class AutoOpRedCloseStartDaniel extends LinearOpMode {

    // ---- Odometry constants ----
    static final double WHEEL_RADIUS_IN = 1.8898;
    static final double GEAR_RATIO = 1.0;
    static final int TICKS_PER_REV = 537;
    static final double INCHES_PER_TICK
            = (2 * Math.PI * WHEEL_RADIUS_IN * GEAR_RATIO) / TICKS_PER_REV;
    static double LATERAL_MULTIPLIER = 1.05;

    // ---- Autonomous driving constants ----
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.8;
    static final double HEADING_THRESHOLD = 2.0;
    static final double DISTANCE_THRESHOLD = 2.0;

    // ---- Loading zone drive parameters ----
    static final double LOADING_ZONE_DRIVE_Y = -76;  // Backward distance
    static final double LOADING_ZONE_DRIVE_X = -18;  // Lateral distance

    // ---- Drive motors ----
    private DcMotorEx fl, fr, bl, br;
    private IMU imu;

    // ---- Accessory motors ----
    private DcMotorEx shootMotor;
    private DcMotor intakeMotor;
    private DcMotor leftIndexMotor;
    private DcMotor rightIndexMotor;

    // ---- Shooter constants ----
    private static final double MOTOR_WARM_UP_POWER = 0.5;
    private static final double SHOOTER_VELOCITY = 1200;
    private static final double INDEXER_ACTIVATION_VELOCITY = 1175;
    private static final double MAX_REVERSE_VELOCITY = 400;  // Maximum reverse velocity when clearing jams

    // ---- Pose tracking ----
    private double x = 0, y = 0, heading = 0;
    private double lastImuYaw = 0;
    private int lastFL, lastFR, lastBL, lastBR;

    // ---- AprilTag variables ----
    final double DESIRED_DISTANCE = 55.0;
    final double SPEED_GAIN = 0.035;
    final double TURN_GAIN = 0.017;
    final double MAX_AUTO_SPEED = 0.55;
    final double MAX_AUTO_TURN = 0.25;
    final double BEARING_OFFSET = 0.0;
    final double YAW_OFFSET = 0.0;
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 24; // Red Alliance AprilTag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // ---- Status tracking ----
    private String currentStatus = "Initializing";

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Initialize AprilTag vision
        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        telemetry.addLine("=== DANIEL RED AUTO OP ===");
        telemetry.addLine();
        telemetry.addLine("Autonomous Sequence:");
        telemetry.addLine("  1. Drive Forward 10\"");
        telemetry.addLine("  2. Scan & Drive to AprilTag");
        telemetry.addLine("  3. First Shot (8 sec)");
        telemetry.addLine("  4. Turn Left 45° & Intake");
        telemetry.addLine("  5. Drive Backward");
        telemetry.addLine("  6. Find AprilTag & Approach");
        telemetry.addLine("  7. Second Shot (8 sec)");
        telemetry.addLine("  8. Strafe Right to Park");
        telemetry.addLine();
        telemetry.addLine("✓ Ready - Press ▶ to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // ==================== AUTONOMOUS SEQUENCE ====================

            // Step 1: Move forward 10 inches
            currentStatus = "Drive Forward 10\"";
            updateStatusDisplay();
            driveDistance(10, 0, DRIVE_SPEED, 5.0);
            currentStatus = "Forward Complete";
            updateStatusDisplay();
            sleep(100);

            // Step 1.5: Quick reverse mode to prepare
            currentStatus = "Quick Reverse Mode";
            updateStatusDisplay();
            reverseMode(0.2);  // Run reverse mode for 200ms
            currentStatus = "Quick Reverse Complete";
            updateStatusDisplay();
            sleep(100);

            // Step 2: Scan for AprilTag and drive to it (scan right first for Red alliance)
            currentStatus = "Scanning for AprilTag";
            updateStatusDisplay();
            if (scanForAprilTag(45, false, 5)) {
                currentStatus = "Tag Found - Driving";
                updateStatusDisplay();
                driveToAprilTag(10.0);
                currentStatus = "AprilTag Complete";
            } else {
                currentStatus = "No AprilTag Found";
            }
            updateStatusDisplay();
            sleep(100);

            // Step 3: Shoot for 8 seconds
            currentStatus = "Shooting 8 sec";
            updateStatusDisplay();
            shootSequence(8.0);
            currentStatus = "Shoot Complete";
            updateStatusDisplay();
            sleep(100);

            // Step 4: Turn left 45 degrees to face balls
            currentStatus = "Turn Left 45°";
            updateStatusDisplay();
            turnToHeading(45, TURN_SPEED, 3.0);
            currentStatus = "Turn Complete";
            updateStatusDisplay();
            sleep(100);

            // Step 5: Drive forward and intake balls
            currentStatus = "Drive Forward & Intake";
            updateStatusDisplay();
            intakeMotor.setPower(1.0);          // Start intake
            shootMotor.setPower(-0.3);          // Reverse shooter slowly to prevent jams
            driveDistance(48, 0, DRIVE_SPEED * 0.7, 7.0);
            intakeMotor.setPower(0);            // Stop intake
            shootMotor.setPower(0);             // Stop shooter
            currentStatus = "Intake Drive Complete";
            updateStatusDisplay();
            sleep(100);

            // Step 6: Drive backward to clear the area
            currentStatus = "Drive Backward";
            updateStatusDisplay();
            driveDistance(-48, 0, DRIVE_SPEED, 5.0);
            currentStatus = "Backward Drive Complete";
            updateStatusDisplay();
            sleep(100);

            // Step 7: Scan for AprilTag and approach for second shot (scan left first)
            currentStatus = "Scanning for AprilTag";
            updateStatusDisplay();
            if (scanForAprilTag(45.0, true, 10.0)) {
                currentStatus = "Tag Found - Driving";
                updateStatusDisplay();
                driveToAprilTag(10.0);
                currentStatus = "AprilTag Complete";
            } else {
                currentStatus = "No AprilTag Found";
            }
            updateStatusDisplay();
            sleep(100);

            // Step 8: Shoot for 8 seconds (second pass)
            currentStatus = "Shooting 8 sec";
            updateStatusDisplay();
            shootSequence(8.0);
            currentStatus = "Shoot Complete";
            updateStatusDisplay();
            sleep(100);

            // Step 9: Strafe right to park
            currentStatus = "Parking: Strafe Right 40\"";
            updateStatusDisplay();
            driveDistance(0, 40, DRIVE_SPEED, 7.0); // Strafe right
            currentStatus = "Strafe Complete";
            updateStatusDisplay();
            sleep(100);

            // Sequence complete
            currentStatus = "✓ SEQUENCE COMPLETE";
            updateStatusDisplay();
            sleep(2000);
        }
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
        // Control Hub: Logo facing UP, USB facing BACKWARD
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
        double imuYaw = -ypr.getYaw(AngleUnit.RADIANS);  // Negate due to USB facing BACKWARD orientation
        double dTheta = wrap(imuYaw - lastImuYaw);
        lastImuYaw = imuYaw;

        int cFL = fl.getCurrentPosition(), cFR = fr.getCurrentPosition();
        int cBL = bl.getCurrentPosition(), cBR = br.getCurrentPosition();
        double dFL = (cFL - lastFL) * INCHES_PER_TICK;
        double dFR = (cFR - lastFR) * INCHES_PER_TICK;
        double dBL = (cBL - lastBL) * INCHES_PER_TICK;
        double dBR = (cBR - lastBR) * INCHES_PER_TICK;
        lastFL = cFL;
        lastFR = cFR;
        lastBL = cBL;
        lastBR = cBR;

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

            // Add minimum power threshold to prevent oscillation
            if (Math.abs(fwd) < 0.08 && Math.abs(str) < 0.08) {
                break;
            }

            setDrivePower(fwd, str, 0);

            telemetry.addData("Status", currentStatus);
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

            telemetry.addData("Status", currentStatus);
            telemetry.addData("🎯 Target Heading", "%.1f°", Math.toDegrees(targetHeadingRad));
            telemetry.addData("🧭 Current Heading", "%.1f°", Math.toDegrees(heading));
            telemetry.addData("↻ Error", "%.1f°", headingErrorDegrees);
            telemetry.addData("⚡ Turn Power", "%.2f", turnPower);
            telemetry.update();
        }

        stopDrive();
    }

    private void driveToAprilTag(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();

        shootMotor.setPower(MOTOR_WARM_UP_POWER);
        telemetry.addLine("🎯 Starting shooter spin-up during drive...");
        telemetry.update();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();

            boolean targetFound = detectAprilTag();

            if (!targetFound) {
                telemetry.addLine("⚠️ Target lost!");
                stopDrive();

                boolean foundAgain = false;
                for (int attempt = 1; attempt <= 2 && opModeIsActive(); attempt++) {
                    telemetry.addLine(String.format("🔍 Rescan attempt %d/2...", attempt));
                    telemetry.update();

                    if (scanForAprilTag(15, true, 5)) { // Scan left first for red
                        telemetry.addLine(String.format("✓ Target found again on attempt %d!", attempt));
                        telemetry.update();
                        foundAgain = true;
                        sleep(200);
                        break;
                    } else {
                        telemetry.addLine(String.format("✗ Attempt %d failed", attempt));
                        telemetry.update();
                        sleep(100);
                    }
                }

                if (!foundAgain) {
                    telemetry.addLine("❌ Target lost permanently after 2 attempts!");
                    telemetry.update();
                    sleep(500);
                    break;
                }
                continue;
            }

            double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double yawError = desiredTag.ftcPose.yaw - YAW_OFFSET;
            double headingError = desiredTag.ftcPose.bearing - BEARING_OFFSET;

            if (Math.abs(rangeError) < 2.0 && Math.abs(yawError) < 3.0 && Math.abs(headingError) < 3.0) {
                telemetry.addLine("✓ Reached target!");
                break;
            }

            double fwd = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, rangeError * SPEED_GAIN));
            double str = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, yawError * SPEED_GAIN));
            double yaw = -Math.max(-MAX_AUTO_TURN, Math.min(MAX_AUTO_TURN, headingError * TURN_GAIN));

            setDrivePower(fwd, str, yaw);

            double shooterVelocity = Math.abs(shootMotor.getVelocity());
            controlShooterVelocity(shooterVelocity, 1.0);

            telemetry.addData("Status", currentStatus);
            telemetry.addData("🏷️ AprilTag", "ID %d", desiredTag.id);
            telemetry.addData("📏 Range", "%.1f\" → %.1f\"", desiredTag.ftcPose.range, DESIRED_DISTANCE);
            telemetry.addData("↔️ Yaw", "%.1f°", desiredTag.ftcPose.yaw);
            telemetry.addData("🧭 Bearing", "%.1f°", desiredTag.ftcPose.bearing);
            telemetry.addData("🎯 Shooter", "%.0f / %.0f ticks/sec", shooterVelocity, SHOOTER_VELOCITY);
            telemetry.update();
        }

        stopDrive();
    }

    private void shootSequence(double durationSeconds) {
        ElapsedTime timer = new ElapsedTime();

        final double INDEXER_ACTIVE_TIME = 1.0;
        final double DELAY_BETWEEN_SHOTS = 1.0;

        shootMotor.setPower(1.0);

        double currentVelocity = Math.abs(shootMotor.getVelocity());
        if (currentVelocity < INDEXER_ACTIVATION_VELOCITY) {
            telemetry.addLine("🎯 Spinning up shooter to target velocity...");
            telemetry.update();

            timer.reset();
            while (opModeIsActive() && timer.seconds() < 3.0) {
                double velocity = Math.abs(shootMotor.getVelocity());
                controlShooterVelocity(velocity, 1.0);

                telemetry.addData("Status", currentStatus);
                telemetry.addData("🎯 Shooter", "%.0f / %.0f ticks/sec", velocity, INDEXER_ACTIVATION_VELOCITY);
                telemetry.addData("Status", velocity >= INDEXER_ACTIVATION_VELOCITY ? "✓ READY" : "⏱️ SPINNING UP");
                telemetry.update();

                if (velocity >= INDEXER_ACTIVATION_VELOCITY) break;
                sleep(50);
            }
        } else {
            telemetry.addLine("✓ Shooter already at target velocity!");
            telemetry.update();
        }

        String[] sequence = {"RIGHT", "LEFT", "RIGHT"};

        for (int shot = 0; shot < 3 && opModeIsActive(); shot++) {
            String currentIndexer = sequence[shot];

            if (shot == 1 || shot == 2) {
                telemetry.addLine(String.format("🔄 Starting intake for shot %d...", shot + 1));
                telemetry.update();
                intakeMotor.setPower(1.0);
                sleep(200);
            }

            telemetry.addLine(String.format("🚀 Shot %d/3: %s INDEXER", shot + 1, currentIndexer));
            telemetry.update();

            if (currentIndexer.equals("RIGHT")) rightIndexMotor.setPower(1.0);
            else leftIndexMotor.setPower(1.0);

            timer.reset();
            while (opModeIsActive() && timer.seconds() < INDEXER_ACTIVE_TIME) {
                double velocity = Math.abs(shootMotor.getVelocity());
                controlShooterVelocity(velocity, 1.0);

                telemetry.addData("Status", currentStatus);
                telemetry.addData("🎯 Shooter", "%.0f ticks/sec", velocity);
                telemetry.addData("📤 Active Indexer", currentIndexer);
                telemetry.addData("🚀 Shot", "%d/3", shot + 1);
                telemetry.addData("⏱️ Shot Time", "%.2f / %.2f sec", timer.seconds(), INDEXER_ACTIVE_TIME);
                if (shot >= 1) telemetry.addData("🔄 Intake", "RUNNING (loading)");
                telemetry.update();
                sleep(50);
            }

            leftIndexMotor.setPower(0);
            rightIndexMotor.setPower(0);

            if (shot < 2) {
                telemetry.addLine(String.format("⏸️ Delay before shot %d...", shot + 2));
                telemetry.update();

                timer.reset();
                while (opModeIsActive() && timer.seconds() < DELAY_BETWEEN_SHOTS) {
                    double velocity = Math.abs(shootMotor.getVelocity());
                    controlShooterVelocity(velocity, 1.0);

                    telemetry.addData("Status", currentStatus);
                    telemetry.addData("🎯 Shooter", "%.0f ticks/sec", velocity);
                    telemetry.addData("⏸️ Delay", "%.2f / %.2f sec", timer.seconds(), DELAY_BETWEEN_SHOTS);
                    if (shot >= 1) telemetry.addData("🔄 Intake", "RUNNING (loading)");
                    telemetry.update();
                    sleep(50);
                }
            }
        }

        shootMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
        intakeMotor.setPower(0);

        telemetry.addLine("✓ Shooting sequence complete!");
        telemetry.update();
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
        setDrivePower(0, 0, 0);
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
        if (visionPortal == null) return;

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
            if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                desiredTag = detection;
                return true;
            }
        }
        return false;
    }

    private boolean scanForAprilTag(double degree, boolean leftFirst, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        final double SCAN_SPEED = 0.2;
        double startHeadingDegrees = Math.toDegrees(heading);

        telemetry.addLine("🔍 Scanning for AprilTag...");
        telemetry.addData("Scan Degree", "%.1f°", degree);
        telemetry.addData("Direction", leftFirst ? "LEFT first" : "RIGHT first");
        telemetry.update();

        String[] directions = leftFirst ? new String[]{"LEFT", "RIGHT"} : new String[]{"RIGHT", "LEFT"};
        double[] turnSpeeds = leftFirst ? new double[]{-SCAN_SPEED, SCAN_SPEED} : new double[]{SCAN_SPEED, -SCAN_SPEED};
        double[] targets = {
            leftFirst ? (startHeadingDegrees - degree) : (startHeadingDegrees + degree),
            startHeadingDegrees,
            leftFirst ? (startHeadingDegrees + degree) : (startHeadingDegrees - degree),
            startHeadingDegrees
        };

        for (int i = 0; i < 4; i++) {
            telemetry.addLine("🔍 Scanning " + (i % 2 == 0 ? directions[i/2] : "Return to Center") + "...");
            telemetry.update();
            timer.reset();
            while (opModeIsActive() && timer.seconds() < timeoutSeconds / 4) {
                if (detectAprilTag()) {
                    stopDrive();
                    telemetry.addLine("✓ AprilTag FOUND!");
                    telemetry.update();
                    sleep(100);
                    return true;
                }
                double currentHeadingDegrees = Math.toDegrees(heading);
                double headingError = wrap(Math.toRadians(targets[i] - currentHeadingDegrees));
                if (Math.abs(Math.toDegrees(headingError)) < 2.0) break;

                setDrivePower(0, 0, (i % 2 == 0) ? turnSpeeds[i/2] : -turnSpeeds[i/2]);
                updatePose();
                sleep(50);
            }
            stopDrive();
            sleep(100);
        }

        telemetry.addLine("❌ No AprilTag found - returning to start...");
        telemetry.update();
        return false;
    }

    private void reverseMode(double duration) {
        ElapsedTime timer = new ElapsedTime();
        telemetry.addLine("⚠️ REVERSE MODE - Clearing jams...");
        telemetry.update();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < duration) {
            double currentShooterVelocity = Math.abs(shootMotor.getVelocity());
            double reversePower = -1.0;
            if (currentShooterVelocity >= MAX_REVERSE_VELOCITY) reversePower = 0;
            else if (currentShooterVelocity >= MAX_REVERSE_VELOCITY * 0.9) reversePower = -0.5;

            shootMotor.setPower(reversePower);
            leftIndexMotor.setPower(-1.0);
            rightIndexMotor.setPower(-1.0);
            intakeMotor.setPower(0.0);
            sleep(50);
        }

        shootMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
        intakeMotor.setPower(0);

        telemetry.addLine("✓ Reverse mode complete!");
        telemetry.update();
        sleep(500);
    }

    private void updateStatusDisplay() {
        telemetry.addData("🤖 Status", currentStatus);
        telemetry.addData("📍 Position", "x=%.1f\", y=%.1f\", h=%.1f°", x, y, Math.toDegrees(heading));
        if (detectAprilTag()) {
            telemetry.addData("🏷️ AprilTag", "✓ ID %d @ %.1f\"", desiredTag.id, desiredTag.ftcPose.range);
        } else {
            telemetry.addData("🏷️ AprilTag", "✗ Not Detected");
        }
        telemetry.update();
    }
}