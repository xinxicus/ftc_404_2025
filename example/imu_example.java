package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * BNO055IMU Complete Usage Example
 * 
 * This example demonstrates:
 * 1. IMU initialization and configuration
 * 2. Reading orientation data (heading, pitch, roll)
 * 3. Field-oriented driving using IMU
 * 4. Heading-based autonomous turns
 * 5. IMU data display on telemetry
 */
@TeleOp(name = "IMU Complete Example")
public class IMUExample extends LinearOpMode {
    
    // Motor declarations for mecanum drive
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    
    // IMU sensor declaration
    private BNO055IMU imu;
    
    // IMU orientation data
    private Orientation angles;
    private double currentHeading = 0;
    private double targetHeading = 0;
    private double headingOffset = 0;
    
    // Drive mode flags
    private boolean fieldOrientedMode = false;
    private boolean lastFieldOrientedButton = false;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Initialize and calibrate IMU
        initializeIMU();
        
        // Wait for start
        telemetry.addData("Status", "IMU Initialized - Ready to start");
        telemetry.addData("Calibration Status", imu.getCalibrationStatus().toString());
        telemetry.update();
        
        waitForStart();
        
        // Reset heading to 0 when starting
        resetHeading();
        
        // Main operation loop
        while (opModeIsActive()) {
            // Update IMU readings
            updateIMUData();
            
            // Handle field-oriented driving toggle
            handleFieldOrientedToggle();
            
            // Process driving controls
            if (fieldOrientedMode) {
                fieldOrientedDrive();
            } else {
                robotOrientedDrive();
            }
            
            // Autonomous heading control with D-pad
            handleAutonomousHeading();
            
            // Display telemetry
            displayTelemetry();
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
        
        // Set motor directions (adjust based on your robot configuration)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * Initialize and configure the BNO055IMU sensor
     */
    private void initializeIMU() {
        // Get IMU from hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        // Create IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        // Set IMU parameters
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // Optional: save/load calibration
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        
        // Initialize IMU with parameters
        imu.initialize(parameters);
        
        // Wait for IMU to calibrate (this can take several seconds)
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        
        // Optional: Wait for calibration to complete
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        
        telemetry.addData("Status", "IMU Calibrated");
        telemetry.update();
    }
    
    /**
     * Update IMU data readings
     */
    private void updateIMUData() {
        // Get current orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        // Update current heading (Z-axis rotation)
        currentHeading = angles.firstAngle - headingOffset;
        
        // Normalize heading to -180 to +180 degrees
        while (currentHeading > 180) currentHeading -= 360;
        while (currentHeading <= -180) currentHeading += 360;
    }
    
    /**
     * Reset heading to zero (set current orientation as new reference)
     */
    private void resetHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        headingOffset = angles.firstAngle;
        currentHeading = 0;
        targetHeading = 0;
    }
    
    /**
     * Handle field-oriented driving mode toggle
     */
    private void handleFieldOrientedToggle() {
        boolean currentFieldOrientedButton = gamepad1.y;
        
        // Toggle field-oriented mode on button press (not hold)
        if (currentFieldOrientedButton && !lastFieldOrientedButton) {
            fieldOrientedMode = !fieldOrientedMode;
            
            // Reset heading when switching to field-oriented mode
            if (fieldOrientedMode) {
                resetHeading();
            }
        }
        
        lastFieldOrientedButton = currentFieldOrientedButton;
    }
    
    /**
     * Robot-oriented driving (standard mecanum drive)
     */
    private void robotOrientedDrive() {
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x;   // Strafe left/right
        double rx = gamepad1.right_stick_x; // Rotate
        
        // Calculate motor powers
        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;
        
        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower), 
                         Math.max(Math.abs(backLeftPower),
                         Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    
    /**
     * Field-oriented driving using IMU heading
     */
    private void fieldOrientedDrive() {
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x;   // Strafe left/right
        double rx = gamepad1.right_stick_x; // Rotate
        
        // Convert robot heading to radians
        double botHeading = Math.toRadians(currentHeading);
        
        // Rotate movement vector by robot heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        
        // Calculate motor powers with field-oriented adjustment
        double frontLeftPower = rotY + rotX + rx;
        double backLeftPower = rotY - rotX + rx;
        double frontRightPower = rotY - rotX - rx;
        double backRightPower = rotY + rotX - rx;
        
        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower), 
                         Math.max(Math.abs(backLeftPower),
                         Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    
    /**
     * Handle autonomous heading control using D-pad
     */
    private void handleAutonomousHeading() {
        // Set target headings with D-pad
        if (gamepad1.dpad_up) {
            targetHeading = 0;    // Face forward
        } else if (gamepad1.dpad_right) {
            targetHeading = -90;  // Face right
        } else if (gamepad1.dpad_down) {
            targetHeading = 180;  // Face backward
        } else if (gamepad1.dpad_left) {
            targetHeading = 90;   // Face left
        }
        
        // Auto-turn to target heading if D-pad was pressed
        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left) {
            turnToHeading(targetHeading, 0.3);
        }
    }
    
    /**
     * Turn robot to specific heading using PID control
     * @param targetHeading Desired heading in degrees
     * @param maxTurnPower Maximum turning power (0.0 to 1.0)
     */
    private void turnToHeading(double targetHeading, double maxTurnPower) {
        // Calculate heading error
        double headingError = targetHeading - currentHeading;
        
        // Normalize error to -180 to +180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        
        // Simple proportional control
        double turnPower = headingError * 0.02; // Adjust this gain as needed
        
        // Limit turn power
        if (Math.abs(turnPower) > maxTurnPower) {
            turnPower = Math.signum(turnPower) * maxTurnPower;
        }
        
        // Apply minimum power to overcome friction (dead zone)
        if (Math.abs(turnPower) > 0.05) {
            if (Math.abs(turnPower) < 0.15) {
                turnPower = Math.signum(turnPower) * 0.15;
            }
            
            // Apply turning power to motors
            frontLeftMotor.setPower(turnPower);
            backLeftMotor.setPower(turnPower);
            frontRightMotor.setPower(-turnPower);
            backRightMotor.setPower(-turnPower);
        }
    }
    
    /**
     * Display IMU and robot status on telemetry
     */
    private void displayTelemetry() {
        // IMU data
        telemetry.addData("Heading", "%.1f°", currentHeading);
        telemetry.addData("Target Heading", "%.1f°", targetHeading);
        telemetry.addData("Pitch", "%.1f°", angles.secondAngle);
        telemetry.addData("Roll", "%.1f°", angles.thirdAngle);
        
        // Drive mode
        telemetry.addData("Drive Mode", fieldOrientedMode ? "Field-Oriented" : "Robot-Oriented");
        
        // Calibration status
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "Yes" : "No");
        telemetry.addData("System Calibrated", imu.isSystemCalibrated() ? "Yes" : "No");
        
        // Controls help
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addData("Y Button", "Toggle Field-Oriented Mode");
        telemetry.addData("D-Pad", "Auto-turn to cardinal directions");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick", "Turn");
        
        telemetry.update();
    }
}
