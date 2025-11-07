package org.firstinspires.ftc.teamcode.error404;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoOp Blue close start 11/1")
public class AutoOpBlueCloseStart extends AutoOpBase {
    // ---- Loading zone drive parameters ----
    static final double LOADING_ZONE_DRIVE_Y = -76;  // Backward distance
    static final double LOADING_ZONE_DRIVE_X = -18;  // Lateral distance

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

    @Override
    protected int getDesiredTagId() {
        return 20;  // Blue alliance AprilTag ID
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        
        // Initialize AprilTag vision (exposure set automatically)
        initAprilTag();
        
        telemetry.addLine("=== RAY AUTO OP 2 ===");
        telemetry.addLine();
        telemetry.addLine("Autonomous Sequence:");
        telemetry.addLine("  1. Drive Forward 10\"");
        telemetry.addLine("  2. Scan & Drive to AprilTag");
        telemetry.addLine("  3. Shoot (8 sec)");
        telemetry.addLine("  4. Reset Pose");
        telemetry.addLine("  5. Strafe Left 40\"");
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
            
            // Step 2: Scan for AprilTag and drive to it
            currentStatus = "Scanning for AprilTag";
            updateStatusDisplay();
            if (scanForAprilTag(45, true, 5)) {
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
            
            // Step 4: Reset pose
            x = 0;
            y = 0;
            heading = 0;
            imu.resetYaw();
            lastImuYaw = 0;
            currentStatus = "Pose Reset";
            updateStatusDisplay();
            sleep(1000);

            // Step 5: Strafe left 40 inches
            currentStatus = "Strafe Left 20";
            updateStatusDisplay();
            driveDistance(0, -40, DRIVE_SPEED, 5.0);
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
    
    /**
     * Turn the robot to a specific heading (absolute angle in degrees)
     * Uses IMU feedback and proportional control for smooth turning
     * 
     * @param targetHeadingDegrees Target heading in degrees (0-360 or any value, will be normalized)
     * @param speed Maximum turning speed (0.0 to 1.0)
     * @param timeoutSeconds Maximum time to attempt the turn
     */
    private void turnToHeading(double targetHeadingDegrees, double speed, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        
        // Convert target heading to radians and normalize to [-PI, PI]
        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);
        targetHeadingRad = wrap(targetHeadingRad);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            
            // Calculate heading error (shortest angular distance)
            double headingError = wrap(targetHeadingRad - heading);
            double headingErrorDegrees = Math.toDegrees(headingError);
            
            // Check if we've reached the target
            if (Math.abs(headingErrorDegrees) < HEADING_THRESHOLD) {
                break;
            }
            
            // Proportional control with speed limiting
            double turnPower = headingError * 0.6;
            turnPower = Math.max(-speed, Math.min(speed, turnPower));
            
            // Add minimum power threshold to overcome static friction
            if (Math.abs(turnPower) < 0.15) {
                // Apply minimum power in the correct direction
                turnPower = Math.signum(turnPower) * 0.15;
            }
            
            // Stop if error is very small to prevent oscillation
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
        
        // Start shooter motor spinning up while driving to save time
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
                
                // Try up to 3 times to rescan for the AprilTag
                boolean foundAgain = false;
                for (int attempt = 1; attempt <= 5 && opModeIsActive(); attempt++) {
                    telemetry.addLine(String.format("🔍 Rescan attempt %d/3...", attempt));
                    telemetry.update();
                    
                    if (scanForAprilTag(15, false, 5)) {
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
                    telemetry.addLine("❌ Target lost permanently after 3 attempts!");
                    telemetry.update();
                    sleep(500);
                    break;
                }
                
                // Target found again, continue to next iteration
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
            
            // Monitor shooter velocity during drive
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
        
        // Shooting configuration
        final double INDEXER_ACTIVE_TIME = 1.0;  // Each indexer active for 1.0 seconds
        final double DELAY_BETWEEN_SHOTS = 1.0;  // 1.0 second delay between shots
        final int LOADBALLS_BEFORE_SHOT_DURATION = 1000;  // Duration to load balls before 3rd shot (ms)
    
        // Shooter motor should already be running from driveToAprilTag
        // But ensure it's running and wait for target velocity if needed
        shootMotor.setPower(1.0);
        
        // Check if shooter is already at speed (likely if called after driveToAprilTag)
        double currentVelocity = Math.abs(shootMotor.getVelocity());
        if (currentVelocity < INDEXER_ACTIVATION_VELOCITY) {
            telemetry.addLine("🎯 Spinning up shooter to target velocity...");
            telemetry.update();
            
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 3.0) {
                double velocity = Math.abs(shootMotor.getVelocity());
                
                // Apply velocity control during spin-up
                controlShooterVelocity(velocity, 1.0);
                
                telemetry.addData("Status", currentStatus);
                telemetry.addData("🎯 Shooter", "%.0f / %.0f ticks/sec", velocity, INDEXER_ACTIVATION_VELOCITY);
                telemetry.addData("Status", velocity >= INDEXER_ACTIVATION_VELOCITY ? "✓ READY" : "⏱️ SPINNING UP");
                telemetry.update();
                
                if (velocity >= INDEXER_ACTIVATION_VELOCITY) {
                    break;
                }
                sleep(50);
            }
        } else {
            telemetry.addLine("✓ Shooter already at target velocity!");
            telemetry.update();
        }
        
        // Shoot 3 times: right, left, right
        String[] sequence = {"RIGHT", "LEFT", "RIGHT"};
        
        for (int shot = 0; shot < 3 && opModeIsActive(); shot++) {
            String currentIndexer = sequence[shot];
            
            // Start intake motor for 2nd and 3rd shots (to load balls while shooting)
            if (shot == 1 || shot == 2) {
                telemetry.addLine(String.format("🔄 Starting intake for shot %d...", shot + 1));
                telemetry.update();
                intakeMotor.setPower(1.0);
                sleep(200);  // Brief delay to ensure intake is running
            }
            
            // Activate the appropriate indexer
            telemetry.addLine(String.format("🚀 Shot %d/3: %s INDEXER", shot + 1, currentIndexer));
            telemetry.update();
            
            if (currentIndexer.equals("RIGHT")) {
                rightIndexMotor.setPower(1.0);
            } else {
                leftIndexMotor.setPower(1.0);
            }
            
            // Keep indexer active for specified time
            timer.reset();
            while (opModeIsActive() && timer.seconds() < INDEXER_ACTIVE_TIME) {
                double velocity = Math.abs(shootMotor.getVelocity());
                
                // Apply velocity control to maintain consistent shooting speed
                controlShooterVelocity(velocity, 1.0);
                
                telemetry.addData("Status", currentStatus);
                telemetry.addData("🎯 Shooter", "%.0f ticks/sec", velocity);
                telemetry.addData("📤 Active Indexer", currentIndexer);
                telemetry.addData("🚀 Shot", "%d/3", shot + 1);
                telemetry.addData("⏱️ Shot Time", "%.2f / %.2f sec", timer.seconds(), INDEXER_ACTIVE_TIME);
                if (shot == 1 || shot == 2) {
                    telemetry.addData("🔄 Intake", "RUNNING (loading)");
                }
                telemetry.update();
                sleep(50);
            }
            
            // Stop the indexers (but keep intake running for shots 2 and 3)
            leftIndexMotor.setPower(0);
            rightIndexMotor.setPower(0);

            
            // Delay between shots (except after the last shot)
            if (shot < 2) {
                telemetry.addLine(String.format("⏸️ Delay before shot %d...", shot + 2));
                telemetry.update();
                
                timer.reset();
                while (opModeIsActive() && timer.seconds() < DELAY_BETWEEN_SHOTS) {
                    double velocity = Math.abs(shootMotor.getVelocity());
                    
                    // Apply velocity control during delays to maintain speed
                    controlShooterVelocity(velocity, 1.0);
                    
                    telemetry.addData("Status", currentStatus);
                    telemetry.addData("🎯 Shooter", "%.0f ticks/sec", velocity);
                    telemetry.addData("⏸️ Delay", "%.2f / %.2f sec", timer.seconds(), DELAY_BETWEEN_SHOTS);
                    // Show intake status during delays
                    if (shot >= 1) {
                        telemetry.addData("🔄 Intake", "RUNNING (loading)");
                    }
                    telemetry.update();
                    sleep(50);
                }
            }
        }
        
        // Stop all motors
        shootMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
        intakeMotor.setPower(0);
        
        telemetry.addLine("✓ Shooting sequence complete!");
        telemetry.update();
    }
    
    /**
     * Control shooter motor velocity to prevent overspeeding and maintain target velocity.
     * This method adjusts motor power based on current velocity to stay within safe limits.
     * 
     * @param currentVelocity Current shooter velocity in ticks/sec (absolute value)
     * @param requestedPower Requested power level (0.0 to 1.0)
     */
    private void controlShooterVelocity(double currentVelocity, double requestedPower) {
        double adjustedPower = requestedPower;
        
        if (currentVelocity >= SHOOTER_VELOCITY) {
            // At or above max velocity - apply light braking to prevent further acceleration
            adjustedPower = -0.1;
        } else if (currentVelocity >= SHOOTER_VELOCITY * 0.95) {
            // Within 5% of max velocity - reduce power proportionally to ease into target
            double velocityRatio = (SHOOTER_VELOCITY - currentVelocity) / (SHOOTER_VELOCITY * 0.05);
            adjustedPower = requestedPower * velocityRatio;
        } else if (currentVelocity >= SHOOTER_VELOCITY * 0.90) {
            // Within 10% of max velocity - cap power to prevent overspeed
            adjustedPower = Math.min(requestedPower, 0.5);
        }
        
        shootMotor.setPower(adjustedPower);
    }
    
    /**
     * Reverse mode - runs shooter, left/right indexer motors in reverse to clear jams
     * Stops intake motor to prevent new balls from entering
     * Based on the reverseMode logic from DriveTestTeleOp_1027.java
     * 
     * @param duration How long to run in reverse mode (seconds)
     */
    private void reverseMode(double duration) {
        ElapsedTime timer = new ElapsedTime();
        
        telemetry.addLine("⚠️ REVERSE MODE - Clearing jams...");
        telemetry.update();
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < duration) {
            updatePose();  // Keep pose tracking updated
            
            // Read current shooter velocity for safety limiting
            double currentShooterVelocity = Math.abs(shootMotor.getVelocity());
            
            // Control shooter motor with velocity limiting (same logic as DriveTestTeleOp_1027)
            double reversePower = -1.0;
            if (currentShooterVelocity >= MAX_REVERSE_VELOCITY) {
                // At or above max reverse velocity - cut power to prevent further acceleration
                reversePower = 0;
            } else if (currentShooterVelocity >= MAX_REVERSE_VELOCITY * 0.95) {
                // Within 5% of max reverse velocity - reduce power proportionally
                double velocityRatio = (MAX_REVERSE_VELOCITY - currentShooterVelocity) / (MAX_REVERSE_VELOCITY * 0.05);
                reversePower = -1.0 * velocityRatio;
            } else if (currentShooterVelocity >= MAX_REVERSE_VELOCITY * 0.90) {
                // Within 10% of max reverse velocity - cap power at -0.5
                reversePower = -0.5;
            }
            
            // Apply motor powers
            shootMotor.setPower(reversePower);
            leftIndexMotor.setPower(-1.0);   // Spin left indexer backwards
            rightIndexMotor.setPower(-1.0);  // Spin right indexer backwards
            intakeMotor.setPower(0.0);        // Stop intake to prevent new balls
            
            // Display status
            telemetry.addData("Status", currentStatus);
            telemetry.addData("Status", "⚠️ REVERSE MODE - CLEARING JAMS");
            telemetry.addData("Shooter", "%.0f ticks/sec (%.2f pwr)", currentShooterVelocity, reversePower);
            telemetry.addData("Left Indexer", "REVERSE");
            telemetry.addData("Right Indexer", "REVERSE");
            telemetry.addData("Intake", "STOPPED");
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), duration);
            
            if (currentShooterVelocity >= MAX_REVERSE_VELOCITY * 0.90) {
                telemetry.addData("⚠️ WARNING", "Approaching max reverse velocity!");
            }
            
            telemetry.update();
            
            sleep(50);
        }
        
        // Stop all motors
        shootMotor.setPower(0);
        leftIndexMotor.setPower(0);
        rightIndexMotor.setPower(0);
        intakeMotor.setPower(0);
        
        telemetry.addLine("✓ Reverse mode complete - jam cleared!");
        telemetry.update();
        sleep(500);
    }
    
}

