package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * Complete AprilTag ftcPose Example
 * 
 * This example demonstrates how to use ALL ftcPose parameters:
 * - Position: x, y, z coordinates
 * - Distance: range and bearing
 * - Orientation: yaw, pitch, roll angles
 * - Practical navigation using these values
 */
@TeleOp(name = "AprilTag Complete Example")
public class AprilTagCompleteExample extends LinearOpMode {
    
    // Motor declarations for mecanum drive
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    
    // Vision processing
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    
    // Navigation parameters
    private static final double TARGET_DISTANCE = 18.0; // Target distance to tag (inches)
    private static final double DISTANCE_TOLERANCE = 2.0; // Acceptable distance range
    private static final double ANGLE_TOLERANCE = 5.0; // Acceptable bearing tolerance (degrees)
    
    // Control gains (adjust these based on your robot)
    private static final double FORWARD_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.02;
    private static final double TURN_GAIN = 0.01;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Initialize vision system
        initializeVision();
        
        telemetry.addData("Status", "Initialized - Ready for AprilTag detection");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Get AprilTag detections
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            
            if (!detections.isEmpty()) {
                // Process the first detected tag
                AprilTagDetection detection = detections.get(0);
                
                // Display all ftcPose parameters
                displayAllFtcPoseData(detection);
                
                // Navigate to the tag using ftcPose data
                navigateToTag(detection);
                
                // Check if we're properly aligned
                checkTagAlignment(detection);
                
            } else {
                telemetry.addData("Status", "No AprilTags detected");
                stopAllMotors();
            }
            
            // Manual override with gamepad
            if (gamepad1.right_bumper) {
                manualDrive();
            }
            
            telemetry.update();
        }
        
        // Clean up
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    /**
     * Initialize all hardware components
     */
    private void initializeHardware() {
        // Map motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        
        // Set motor directions (adjust for your robot)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * Initialize vision processing system
     */
    private void initializeVision() {
        // Create AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        
        // Create vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }
    
    /**
     * Display all available ftcPose parameters
     */
    private void displayAllFtcPoseData(AprilTagDetection detection) {
        telemetry.addData("=== Tag ID ===", detection.id);
        telemetry.addLine();
        
        // Position parameters (translation)
        telemetry.addData("X (Left/Right)", "%.1f inches", detection.ftcPose.x);
        telemetry.addData("Y (Up/Down)", "%.1f inches", detection.ftcPose.y);
        telemetry.addData("Z (Forward)", "%.1f inches", detection.ftcPose.z);
        telemetry.addLine();
        
        // Distance and bearing (convenient parameters)
        telemetry.addData("Range (Distance)", "%.1f inches", detection.ftcPose.range);
        telemetry.addData("Bearing (Angle)", "%.1f degrees", detection.ftcPose.bearing);
        telemetry.addLine();
        
        // Orientation parameters (rotation)
        telemetry.addData("Yaw (Rotation)", "%.1f degrees", detection.ftcPose.yaw);
        telemetry.addData("Pitch (Tilt)", "%.1f degrees", detection.ftcPose.pitch);
        telemetry.addData("Roll (Lean)", "%.1f degrees", detection.ftcPose.roll);
        telemetry.addLine();
        
        // Analysis
        telemetry.addData("Tag Quality", getTagQuality(detection));
        telemetry.addData("Distance Status", getDistanceStatus(detection.ftcPose.range));
        telemetry.addData("Alignment Status", getAlignmentStatus(detection.ftcPose.bearing));
    }
    
    /**
     * Navigate to the AprilTag using ftcPose data
     */
    private void navigateToTag(AprilTagDetection detection) {
        // Calculate movement corrections based on ftcPose
        double forwardPower = 0;
        double strafePower = 0;
        double turnPower = 0;
        
        // Forward/backward control using range
        double distanceError = detection.ftcPose.range - TARGET_DISTANCE;
        if (Math.abs(distanceError) > DISTANCE_TOLERANCE) {
            forwardPower = distanceError * FORWARD_GAIN;
        }
        
        // Left/right strafe control using x position
        if (Math.abs(detection.ftcPose.x) > 2.0) {
            strafePower = -detection.ftcPose.x * STRAFE_GAIN; // Negative to correct
        }
        
        // Turning control using bearing
        if (Math.abs(detection.ftcPose.bearing) > ANGLE_TOLERANCE) {
            turnPower = -detection.ftcPose.bearing * TURN_GAIN; // Negative to correct
        }
        
        // Apply power limits
        forwardPower = Math.max(-0.5, Math.min(0.5, forwardPower));
        strafePower = Math.max(-0.3, Math.min(0.3, strafePower));
        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
        
        // Apply mecanum drive kinematics
        double frontLeftPower = forwardPower + strafePower + turnPower;
        double frontRightPower = forwardPower - strafePower - turnPower;
        double backLeftPower = forwardPower - strafePower + turnPower;
        double backRightPower = forwardPower + strafePower - turnPower;
        
        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        
        // Display navigation info
        telemetry.addData("=== Navigation ===", "");
        telemetry.addData("Forward Power", "%.2f", forwardPower);
        telemetry.addData("Strafe Power", "%.2f", strafePower);
        telemetry.addData("Turn Power", "%.2f", turnPower);
    }
    
    /**
     * Check if robot is properly aligned with the tag
     */
    private void checkTagAlignment(AprilTagDetection detection) {
        boolean distanceGood = Math.abs(detection.ftcPose.range - TARGET_DISTANCE) < DISTANCE_TOLERANCE;
        boolean bearingGood = Math.abs(detection.ftcPose.bearing) < ANGLE_TOLERANCE;
        boolean positionGood = Math.abs(detection.ftcPose.x) < 2.0;
        
        telemetry.addData("=== Alignment Check ===", "");
        telemetry.addData("Distance OK", distanceGood ? "✓" : "✗");
        telemetry.addData("Bearing OK", bearingGood ? "✓" : "✗");
        telemetry.addData("Position OK", positionGood ? "✓" : "✗");
        
        if (distanceGood && bearingGood && positionGood) {
            telemetry.addData("STATUS", "🎯 PERFECTLY ALIGNED!");
            stopAllMotors();
        }
    }
    
    /**
     * Manual drive override
     */
    private void manualDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;
        
        frontLeftMotor.setPower(frontLeftPower * 0.7);
        frontRightMotor.setPower(frontRightPower * 0.7);
        backLeftMotor.setPower(backLeftPower * 0.7);
        backRightMotor.setPower(backRightPower * 0.7);
        
        telemetry.addData("Mode", "Manual Drive");
    }
    
    /**
     * Stop all motors
     */
    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    
    /**
     * Analyze tag detection quality
     */
    private String getTagQuality(AprilTagDetection detection) {
        double range = detection.ftcPose.range;
        double yaw = Math.abs(detection.ftcPose.yaw);
        double pitch = Math.abs(detection.ftcPose.pitch);
        
        if (range > 48 || yaw > 45 || pitch > 30) {
            return "❌ Poor";
        } else if (range > 36 || yaw > 30 || pitch > 20) {
            return "⚠️ Fair";
        } else if (range > 24 || yaw > 15 || pitch > 10) {
            return "✅ Good";
        } else {
            return "🌟 Excellent";
        }
    }
    
    /**
     * Get distance status
     */
    private String getDistanceStatus(double range) {
        if (range < 12) {
            return "🔴 Too Close";
        } else if (range > 36) {
            return "🔵 Too Far";
        } else if (Math.abs(range - TARGET_DISTANCE) < DISTANCE_TOLERANCE) {
            return "🟢 Perfect";
        } else {
            return "🟡 Adjusting";
        }
    }
    
    /**
     * Get alignment status
     */
    private String getAlignmentStatus(double bearing) {
        if (Math.abs(bearing) < ANGLE_TOLERANCE) {
            return "🟢 Aligned";
        } else if (bearing > 0) {
            return "🔄 Turn Left";
        } else {
            return "🔄 Turn Right";
        }
    }
}
