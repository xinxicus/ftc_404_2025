// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
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

@TeleOp(name="TeleOp test 10/24")
public class DriveTestTeleOp_1024 extends LinearOpMode {
    // ---- fill with your real values later (for pose readout, optional) ----
    static final double WHEEL_RADIUS_IN = 1.8898; // 48mm
    static final double GEAR_RATIO = 1.0;
    static final int    TICKS_PER_REV = 537;
    static final double INCHES_PER_TICK =
            (2 * Math.PI * WHEEL_RADIUS_IN * GEAR_RATIO) / TICKS_PER_REV;
    static double LATERAL_MULTIPLIER = 1.05;

    private DcMotorEx fl, fr, bl, br;
    private IMU imu;
    
    // Accessory motors
    private DcMotorEx shootMotor;  // Using DcMotorEx to get velocity readings
    private DcMotor intakeMotor;
    private DcMotor leftIndexMotor;
    private DcMotor rightIndexMotor;
    
    // Shooter and indexer safety constants
    private static final double SHOOTER_VELOCITY = 1080;  // Target shooter velocity (ticks/sec) - indexers activate when reached, motor limited at this speed
    private static final double MAX_REVERSE_VELOCITY = 800;  // Maximum reverse velocity (ticks/sec) when clearing jams (adjust based on your shooter)
    private static final double VELOCITY_TOLERANCE = 0;     // Velocity tolerance for "at speed" detection (hysteresis to prevent jitter)
    
    // Shooter state tracking
    private boolean shooterWasAtSpeed = false;  // Track previous state for hysteresis
    private boolean shooterCurrentlyAtSpeed = false;  // Current state for telemetry display
    private double currentShooterVelocity = 0;  // Current velocity (ticks/sec) for telemetry display
    
    // Active braking for shooter motor
    private double lastShooterTrigger = 0;  // Track previous trigger value to detect release
    private long brakingStartTime = 0;  // Time when braking started
    private static final long BRAKING_DURATION_MS = 1100;  // How long to apply reverse power (milliseconds)
    private static final double BRAKING_POWER = -0.8;  // How hard to brake (range: -0.3 to -1.0, more negative = stronger braking)

    // optional: simple pose integration (encoders + IMU)
    private double x=0, y=0, heading=0;
    private double lastImuYaw=0;
    private int lastFL, lastFR, lastBL, lastBR;

    private boolean fieldCentric = true; // B to toggle
    private boolean prevB=false, prevY=false;

    // AprilTag variables and constants
    final double DESIRED_DISTANCE = 55.0; 
    final double SPEED_GAIN =   0.035 ;   
    final double TURN_GAIN  =   0.017;   
    final double MAX_AUTO_SPEED = 0.75;   
    final double MAX_AUTO_TURN  = 0.25;
    // Camera offset compensation (camera is not at robot center)
    final double BEARING_OFFSET = 14.0;  // degrees - adjust robot angle offset
    final double YAW_OFFSET = 11.0;       // degrees - adjust lateral alignment offset
    private static final boolean USE_WEBCAM = true;  
    private static final int DESIRED_TAG_ID = 20;    
    private VisionPortal visionPortal;               
    private AprilTagProcessor aprilTag;              
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {
        // ---- map hardware ----
        fl = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // >>> Right side reversed <<<
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // open-loop: read encoders, control by power
        for (DcMotorEx m : new DcMotorEx[]{fl,fr,bl,br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // IMU (adjust orientation to match your mounting)
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
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIndexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIndexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set motor modes - shooter uses RUN_USING_ENCODER to read velocity
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize AprilTag
        initAprilTag();
        if (USE_WEBCAM)
            setManualExposure(6, 250);

        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();

        telemetry.addLine("Ready. Press ▶");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // AprilTag detection
            boolean targetFound = detectAprilTag();
            displayAprilTagStatus(targetFound);
            // --- (optional) pose update ---
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            double imuYaw = ypr.getYaw(AngleUnit.RADIANS);
            double dTheta = wrap(imuYaw - lastImuYaw);
            lastImuYaw = imuYaw;

            int cFL = fl.getCurrentPosition(), cFR = fr.getCurrentPosition();
            int cBL = bl.getCurrentPosition(), cBR = br.getCurrentPosition();
            double dFL = (cFL - lastFL) * INCHES_PER_TICK;
            double dFR = (cFR - lastFR) * INCHES_PER_TICK;
            double dBL = (cBL - lastBL) * INCHES_PER_TICK;
            double dBR = (cBR - lastBR) * INCHES_PER_TICK;
            lastFL = cFL; lastFR = cFR; lastBL = cBL; lastBR = cBR;

            double dxR = (dFL + dFR + dBL + dBR)/4.0;
            double dyR = (-dFL + dFR + dBL - dBR)/4.0 * LATERAL_MULTIPLIER;

            double hMid = heading + dTheta/2.0, c = Math.cos(hMid), s = Math.sin(hMid);
            x +=  dxR * c - dyR * s;
            y +=  dxR * s + dyR * c;
            heading = wrap(heading + dTheta);

            // --- driving: check for AprilTag auto-drive or manual control ---
            double fwd, str, yaw;
            
            // Try AprilTag auto-drive first (A button on either controller)
            double[] autoDriveCommands = getAutoDriveCommands(targetFound);
            if (autoDriveCommands != null) {
                // AprilTag auto-drive mode is active
                fwd = autoDriveCommands[0];
                str = autoDriveCommands[1];
                yaw = autoDriveCommands[2];
            } else {
                // Manual control mode - combine inputs from both controllers
                double y1 = -gamepad1.left_stick_y;
                double x1 = gamepad1.left_stick_x;
                double rx1 = gamepad1.right_stick_x;
                
                double y2 = -gamepad2.left_stick_y;
                double x2 = gamepad2.left_stick_x;
                double rx2 = gamepad2.right_stick_x;
                
                // Use whichever controller has greater input magnitude
                fwd = (Math.abs(y1) > Math.abs(y2)) ? y1 : y2;
                str = (Math.abs(x1) > Math.abs(x2)) ? x1 : x2;
                yaw = (Math.abs(rx1) > Math.abs(rx2)) ? rx1 : rx2;
                
                // B toggles field-centric on either controller; Y resets yaw on either controller
                boolean curB = gamepad1.b || gamepad2.b;
                if (curB && !prevB) fieldCentric = !fieldCentric;
                prevB = curB;

                boolean curY = gamepad1.y || gamepad2.y;
                if (curY && !prevY) { imu.resetYaw(); heading = 0; }
                prevY = curY;

                if (fieldCentric) {
                    double cc = Math.cos(heading), ss = Math.sin(heading);
                    double f2 = fwd * cc - str * ss;
                    double s2 = fwd * ss + str * cc;
                    fwd = f2; str = s2;
                }
                
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f", fwd, str, yaw);
            }

            // mecanum mix (open-loop power)
            double flPow = fwd + str + yaw;
            double blPow = fwd - str + yaw;
            double frPow = fwd - str - yaw;
            double brPow = fwd + str - yaw;

            // normalize to [-1,1]
            double max = Math.max(1.0, Math.max(Math.abs(flPow),
                        Math.max(Math.abs(blPow), Math.abs(frPow))));
            max = Math.max(max, Math.abs(brPow));
            flPow/=max; blPow/=max; frPow/=max; brPow/=max;

            fl.setPower(flPow);
            bl.setPower(blPow);
            fr.setPower(frPow);
            br.setPower(brPow);
            
            // Handle accessory motors
            handleAccessoryMotors();

            telemetry.addData("Drive", fieldCentric ? "FIELD" : "ROBOT");
            telemetry.addData("Pose", "x=%.1f in, y=%.1f in, h=%.1f°",
                    x, y, Math.toDegrees(heading));
            telemetry.addData("Hint", "A:AprilTag, B:Field/Robot, Y:Reset, L2:Reverse(Clear Jams)");
            telemetry.addData("", "");
            telemetry.addData("Accessories", "");
            boolean reverseActive = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
            if (reverseActive) {
                if (currentShooterVelocity >= MAX_REVERSE_VELOCITY * 0.90) {
                    telemetry.addData("  MODE", "⚠️ REVERSE - VELOCITY LIMITED! %.0f/%.0f ticks/sec", currentShooterVelocity, MAX_REVERSE_VELOCITY);
                } else {
                    telemetry.addData("  MODE", "⚠️ REVERSE - ALL MOTORS REVERSING! (%.0f ticks/sec)", currentShooterVelocity);
                }
            }
            
            // Display shooter velocity and status (using actual state from handleAccessoryMotors)
            if (currentShooterVelocity >= SHOOTER_VELOCITY * 0.95) {
                // Show warning when approaching or at max velocity
                telemetry.addData("  Shooter", "⚠️ MAX VELOCITY - %.0f/%.0f ticks/sec (%.2f pwr)", currentShooterVelocity, SHOOTER_VELOCITY, shootMotor.getPower());
                if (shooterCurrentlyAtSpeed) {
                    telemetry.addData("  Indexers", "ENABLED - Press L1/R1");
                } else {
                    telemetry.addData("  Indexers", "LOCKED - Need %.0f ticks/sec", SHOOTER_VELOCITY);
                }
            } else if (shooterCurrentlyAtSpeed) {
                telemetry.addData("  Shooter", "✓ READY - %.0f ticks/sec (%.2f pwr)", currentShooterVelocity, shootMotor.getPower());
                telemetry.addData("  Indexers", "ENABLED - Press L1/R1");
            } else if (shootMotor.getPower() > 0.1) {
                telemetry.addData("  Shooter", "⏱ SPINNING UP - %.0f/%.0f ticks/sec", currentShooterVelocity, SHOOTER_VELOCITY);
                telemetry.addData("  Indexers", "LOCKED - Need %.0f ticks/sec", SHOOTER_VELOCITY);
            } else {
                telemetry.addData("  Shooter", "%.2f pwr (%.0f ticks/sec)", shootMotor.getPower(), currentShooterVelocity);
                telemetry.addData("  Indexers", "LOCKED - Shooter not active");
            }
            
            // Debug: Show encoder position to verify encoder is working
            telemetry.addData("DEBUG Shooter Encoder", "%d ticks", shootMotor.getCurrentPosition());
            
            boolean leftIndexRequested = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean rightIndexRequested = gamepad1.right_bumper || gamepad2.right_bumper;
            telemetry.addData("  Left Index", leftIndexRequested ? (shooterCurrentlyAtSpeed ? "ON" : "WAITING") : "OFF");
            telemetry.addData("  Right Index", rightIndexRequested ? (shooterCurrentlyAtSpeed ? "ON" : "WAITING") : "OFF");
            telemetry.addData("  Intake", "%.2f", intakeMotor.getPower());
            telemetry.update();
        }
    }

    private static double wrap(double a){
        while (a <= -Math.PI) a += 2*Math.PI;
        while (a >   Math.PI) a -= 2*Math.PI;
        return a;
    }
    
    /**
     * Get the current shooter motor velocity in ticks per second
     * Returns 0 if there's any issue reading velocity
     */
    private double getShooterVelocity() {
        try {
            double velocityTicksPerSec = shootMotor.getVelocity();
            
            // Debug: store raw values for telemetry
            telemetry.addData("DEBUG Shooter Velocity", "%.1f ticks/sec", velocityTicksPerSec);
            
            return Math.abs(velocityTicksPerSec);
        } catch (Exception e) {
            // Return 0 if there's any issue reading velocity
            telemetry.addData("DEBUG ERROR", e.getMessage());
            return 0;
        }
    }
    
    /**
     * Handle intake and shooter motor controls via bumpers and triggers
     * Both gamepad1 and gamepad2 can control accessories
     * L2 (left trigger) reverses ALL motors (intake, shooter, and BOTH indexers) to clear jams
     * In normal mode: Indexers only activate when shooter reaches minimum velocity (velocity-based)
     */
    private void handleAccessoryMotors() {
        // Check if L2 (left trigger) is pressed for reverse mode (jam clearing)
        boolean reverseMode = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        
        // R2 (right trigger) controls shooter motor - use max of both controllers
        // L2 (left trigger) reverses shooter for jam clearing
        double shooterTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        long currentTime = System.currentTimeMillis();
        
        // Read actual shooter velocity (ticks/sec) from encoder - do this once per loop
        currentShooterVelocity = getShooterVelocity();
        
        if (reverseMode) {
            // Limit reverse velocity to prevent motor damage during jam clearing
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
            
            shootMotor.setPower(reversePower);
            shooterWasAtSpeed = false;  // Reset hysteresis state when reversing
            brakingStartTime = 0;  // Cancel any active braking
        } else {
            // Detect when trigger is released (was pressed, now released)
            boolean triggerJustReleased = (lastShooterTrigger > 0.1 && shooterTrigger < 0.05);
            
            if (triggerJustReleased) {
                // Start active braking when trigger is released
                brakingStartTime = currentTime;
            }
            
            // Cancel braking if user presses trigger again during braking period
            if (shooterTrigger > 0.1 && brakingStartTime > 0) {
                brakingStartTime = 0;  // Cancel braking immediately
            }
            
            // Check if we're in the braking period
            if (brakingStartTime > 0 && (currentTime - brakingStartTime) < BRAKING_DURATION_MS) {
                // Apply reverse power to brake quickly
                shootMotor.setPower(BRAKING_POWER);
            } else {
                // Either normal operation or braking period ended
                brakingStartTime = 0;  // Clear braking state
                
                // Limit power if velocity is approaching or exceeding maximum
                double adjustedPower = shooterTrigger;
                if (currentShooterVelocity >= SHOOTER_VELOCITY) {
                    // At or above max velocity - apply negative power to actively brake
                    adjustedPower = -0.1;  // Light braking to prevent further acceleration
                } else if (currentShooterVelocity >= SHOOTER_VELOCITY * 0.95) {
                    // Within 5% of max velocity - reduce power proportionally
                    double velocityRatio = (SHOOTER_VELOCITY - currentShooterVelocity) / (SHOOTER_VELOCITY * 0.05);
                    adjustedPower = shooterTrigger * velocityRatio;
                } else if (currentShooterVelocity >= SHOOTER_VELOCITY * 0.90) {
                    // Within 10% of max velocity - cap power to prevent overspeed
                    adjustedPower = Math.min(shooterTrigger, 0.5);
                }
                
                shootMotor.setPower(adjustedPower);
            }
        }
        
        lastShooterTrigger = shooterTrigger;  // Remember for next iteration
        
        // Use hysteresis to prevent jitter when velocity fluctuates near threshold
        // If currently OFF: need to reach SHOOTER_VELOCITY to turn ON
        // If currently ON: stay ON until drops below (SHOOTER_VELOCITY - VELOCITY_TOLERANCE)
        if (reverseMode || shooterTrigger < 0.05) {
            // Reset state if not actively shooting
            shooterCurrentlyAtSpeed = false;
            shooterWasAtSpeed = false;
        } else if (shooterWasAtSpeed) {
            // Already at speed, only turn off if drops significantly below threshold
            shooterCurrentlyAtSpeed = currentShooterVelocity >= (SHOOTER_VELOCITY - VELOCITY_TOLERANCE);
            shooterWasAtSpeed = shooterCurrentlyAtSpeed;
        } else {
            // Not at speed yet, need to reach full threshold
            shooterCurrentlyAtSpeed = currentShooterVelocity >= SHOOTER_VELOCITY;
            shooterWasAtSpeed = shooterCurrentlyAtSpeed;
        }
        
        // L1 (left bumper) controls left index motor - only if shooter is at speed
        // L2 (left trigger, reverse mode) overrides and spins indexers in reverse to clear jams
        boolean leftBumperPressed = gamepad1.left_bumper || gamepad2.left_bumper;
        if (reverseMode) {
            // In reverse mode, spin left indexer backwards to clear jams
            leftIndexMotor.setPower(-1.0);
        } else if (leftBumperPressed && shooterCurrentlyAtSpeed) {
            leftIndexMotor.setPower(1.0);
        } else {
            leftIndexMotor.setPower(0);
        }

        // R1 (right bumper) controls right index motor - only if shooter is at speed
        // L2 (left trigger, reverse mode) overrides and spins indexers in reverse to clear jams
        boolean rightBumperPressed = gamepad1.right_bumper || gamepad2.right_bumper;
        if (reverseMode) {
            // In reverse mode, spin right indexer backwards to clear jams
            rightIndexMotor.setPower(-1.0);
        } else if (rightBumperPressed && shooterCurrentlyAtSpeed) {
            rightIndexMotor.setPower(1.0);
        } else {
            rightIndexMotor.setPower(0);
        }

        // X button controls intake motor - use OR logic for both controllers
        // L2 (left trigger) stops intake in reverse mode
        boolean intakePressed = gamepad1.x || gamepad2.x;
        if (reverseMode) {
            intakeMotor.setPower(0.0);  // Stop intake when L2 is pressed
        } else {
            intakeMotor.setPower(intakePressed ? 1.0 : 0.0);
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
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
        
        if (!isStopRequested())
        {
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
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
    
    /**
     * Detect AprilTag and set desiredTag if found
     * @return true if the desired tag is found, false otherwise
     */
    private boolean detectAprilTag() {
        desiredTag = null;
        
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    desiredTag = detection;
                    return true;  // Target found
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        
        return false;  // No target found
    }
    
    /**
     * Display AprilTag detection status to telemetry
     * @param targetFound whether a valid target was detected
     */
    private void displayAprilTagStatus(boolean targetFound) {
        if (targetFound) {
            telemetry.addData("\n>","HOLD A Button to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }
    }
    
    /**
     * Calculate auto-drive commands based on AprilTag position
     * @param targetFound whether a valid target was detected
     * @return double array [fwd, str, yaw] if auto-drive is active, null otherwise
     */
    private double[] getAutoDriveCommands(boolean targetFound) {
        // Either controller can activate AprilTag auto-drive with A button
        if ((gamepad1.a || gamepad2.a) && targetFound) {
            // AprilTag auto-drive mode (activated by A button)
            // Apply camera offset compensation (camera is not at robot center)
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double yawError = (desiredTag.ftcPose.yaw - YAW_OFFSET);        // Compensate for camera lateral offset
            double headingError = (desiredTag.ftcPose.bearing - BEARING_OFFSET);  // Compensate for camera angular offset
            
            double fwd = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double str = Range.clip(yawError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double yaw = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            
            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f", fwd, str, yaw);
            telemetry.addData("DEBUG Range", "%.1f\" (target: %.1f\")", desiredTag.ftcPose.range, DESIRED_DISTANCE);
            telemetry.addData("DEBUG Yaw", "%.1f° (offset: %.1f°, error: %.1f°)", desiredTag.ftcPose.yaw, YAW_OFFSET, yawError);
            telemetry.addData("DEBUG Bearing", "%.1f° (offset: %.1f°, error: %.1f°)", desiredTag.ftcPose.bearing, BEARING_OFFSET, headingError);
            
            return new double[]{fwd, str, yaw};
        }
        
        return null;  // Auto-drive not active
    }
    
}