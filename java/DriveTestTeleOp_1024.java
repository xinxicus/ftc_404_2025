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
    private static final double MIN_SHOOTER_RPM = 20;  // Minimum shooter RPM before indexers can activate (adjust based on your shooter)
    private static final double RPM_TOLERANCE = 0;     // RPM tolerance for "at speed" detection (hysteresis to prevent jitter)
    
    // Shooter state tracking
    private boolean shooterWasAtSpeed = false;  // Track previous state for hysteresis
    private boolean shooterCurrentlyAtSpeed = false;  // Current state for telemetry display
    private double currentShooterRPM = 0;  // Current RPM for telemetry display

    // optional: simple pose integration (encoders + IMU)
    private double x=0, y=0, heading=0;
    private double lastImuYaw=0;
    private int lastFL, lastFR, lastBL, lastBR;

    private boolean fieldCentric = true; // B to toggle
    private boolean prevB=false, prevY=false;

    // AprilTag variables and constants
    final double DESIRED_DISTANCE = 48.0; 
    final double SPEED_GAIN =   0.035 ;   
    final double TURN_GAIN  =   0.017;   
    final double MAX_AUTO_SPEED = 0.75;   
    final double MAX_AUTO_TURN  = 0.25;  
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
            // AprilTag detection variables
            boolean targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Display AprilTag status
            if (targetFound) {
                telemetry.addData("\n>","HOLD A Button to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }
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
            
            // Either controller can activate AprilTag auto-drive with A button
            if ((gamepad1.a || gamepad2.a) && targetFound) {
                // AprilTag auto-drive mode (activated by A button)
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double yawError = (desiredTag.ftcPose.yaw);
                double headingError = desiredTag.ftcPose.bearing;
                
                fwd = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                str = Range.clip(yawError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                yaw = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f", fwd, str, yaw);
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
            telemetry.addData("Hint", "A:AprilTag, B:Field/Robot, Y:Reset, X:Reverse(Clear Jams)");
            telemetry.addData("", "");
            telemetry.addData("Accessories", "");
            boolean reverseActive = gamepad1.x || gamepad2.x;
            if (reverseActive) {
                telemetry.addData("  MODE", "⚠️ REVERSE - ALL MOTORS REVERSING!");
            }
            
            // Display shooter RPM and status (using actual state from handleAccessoryMotors)
            if (shooterCurrentlyAtSpeed) {
                telemetry.addData("  Shooter", "✓ READY - %.0f RPM (%.2f pwr)", currentShooterRPM, shootMotor.getPower());
                telemetry.addData("  Indexers", "ENABLED - Press L1/R1");
            } else if (shootMotor.getPower() > 0.1) {
                telemetry.addData("  Shooter", "⏱ SPINNING UP - %.0f/%.0f RPM", currentShooterRPM, MIN_SHOOTER_RPM);
                telemetry.addData("  Indexers", "LOCKED - Need %.0f RPM", MIN_SHOOTER_RPM);
            } else {
                telemetry.addData("  Shooter", "%.2f pwr (%.0f RPM)", shootMotor.getPower(), currentShooterRPM);
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
     * Get the current shooter motor RPM
     * Returns 0 if motor type is not configured
     */
    private double getShooterRPM() {
        try {
            double velocityTicksPerSec = shootMotor.getVelocity();
            double ticksPerRev = shootMotor.getMotorType().getTicksPerRev();
            
            // Debug: store raw values for telemetry
            telemetry.addData("DEBUG Shooter Velocity", "%.1f ticks/sec", velocityTicksPerSec);
            telemetry.addData("DEBUG Ticks Per Rev", "%.0f", ticksPerRev);
            telemetry.addData("DEBUG Motor Type", shootMotor.getMotorType().toString());
            
            if (ticksPerRev == 0) {
                telemetry.addData("DEBUG ERROR", "Ticks per rev is ZERO!");
                return 0;  // Prevent division by zero
            }
            // Correct RPM formula: (ticks/sec * 60 sec/min) / (ticks/rev) = revolutions/min
            return Math.abs(velocityTicksPerSec * 60.0 / ticksPerRev);
        } catch (Exception e) {
            // Return 0 if there's any issue reading velocity
            telemetry.addData("DEBUG ERROR", e.getMessage());
            return 0;
        }
    }
    
    /**
     * Handle intake and shooter motor controls via bumpers and triggers
     * Both gamepad1 and gamepad2 can control accessories
     * X button reverses ALL motors (intake, shooter, and BOTH indexers) to clear jams
     * In normal mode: Indexers only activate when shooter reaches minimum RPM (velocity-based)
     */
    private void handleAccessoryMotors() {
        // Check if X button is pressed for reverse mode (jam clearing)
        boolean reverseMode = gamepad1.x || gamepad2.x;
        
        // R2 (right trigger) controls shooter motor - use max of both controllers
        // X button reverses shooter for jam clearing
        double shooterTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        if (reverseMode) {
            shootMotor.setPower(-1.0);  // Full reverse when X is pressed
            shooterWasAtSpeed = false;  // Reset hysteresis state when reversing
        } else {
            shootMotor.setPower(shooterTrigger);
        }
        
        // Read actual shooter velocity (RPM) from encoder
        currentShooterRPM = getShooterRPM();
        
        // Use hysteresis to prevent jitter when RPM fluctuates near threshold
        // If currently OFF: need to reach MIN_SHOOTER_RPM to turn ON
        // If currently ON: stay ON until drops below (MIN_SHOOTER_RPM - RPM_TOLERANCE)
        if (reverseMode || shooterTrigger < 0.05) {
            // Reset state if not actively shooting
            shooterCurrentlyAtSpeed = false;
            shooterWasAtSpeed = false;
        } else if (shooterWasAtSpeed) {
            // Already at speed, only turn off if drops significantly below threshold
            shooterCurrentlyAtSpeed = currentShooterRPM >= (MIN_SHOOTER_RPM - RPM_TOLERANCE);
            shooterWasAtSpeed = shooterCurrentlyAtSpeed;
        } else {
            // Not at speed yet, need to reach full threshold
            shooterCurrentlyAtSpeed = currentShooterRPM >= MIN_SHOOTER_RPM;
            shooterWasAtSpeed = shooterCurrentlyAtSpeed;
        }
        
        // L1 (left bumper) controls left index motor - only if shooter is at speed
        // X button (reverse mode) overrides and spins indexers in reverse to clear jams
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
        // X button (reverse mode) overrides and spins indexers in reverse to clear jams
        boolean rightBumperPressed = gamepad1.right_bumper || gamepad2.right_bumper;
        if (reverseMode) {
            // In reverse mode, spin right indexer backwards to clear jams
            rightIndexMotor.setPower(-1.0);
        } else if (rightBumperPressed && shooterCurrentlyAtSpeed) {
            rightIndexMotor.setPower(1.0);
        } else {
            rightIndexMotor.setPower(0);
        }

        // L2 (left trigger) controls intake motor - use max of both controllers
        // X button reverses intake for jam clearing
        double intakeTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        if (reverseMode) {
            intakeMotor.setPower(-1.0);  // Full reverse when X is pressed
        } else {
            intakeMotor.setPower(intakeTrigger);
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
    
}