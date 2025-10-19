package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Driver Test TeleOp", group = "Tests")
public class DriverTestTeleOp extends LinearOpMode {

    // Mecanum drive motors
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    
    // Accessory motors
    private DcMotor shootMotor;
    private DcMotor intakeMotor;
    private DcMotor leftIndexMotor;
    private DcMotor rightIndexMotor;

    // Constants
    private static final double SHOOTER_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double STRAFE_MULTIPLIER = 1.1; // Counteract imperfect strafing

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        displayInitializedTelemetry();
        
        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleMecanumDrive();
            handleAccessoryMotors();
            updateTelemetry();
        }
    }

    /**
     * Initialize all motors and configure their directions
     */
    private void initializeHardware() {
        // Initialize mecanum drive motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse left side motors for mecanum drive
        // Adjust these if your robot moves backwards when commanded forward
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize accessory motors
        shootMotor = hardwareMap.dcMotor.get("shootMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftIndexMotor = hardwareMap.dcMotor.get("leftIndexMotor");
        rightIndexMotor = hardwareMap.dcMotor.get("rightIndexMotor");

        // Reverse left index motor direction
        leftIndexMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIndexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Display telemetry information when robot is initialized
     */
    private void displayInitializedTelemetry() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to begin");
        telemetry.addData("Controls", "Both Controllers Supported");
        telemetry.addData("  Left Stick", "Drive (Y/X)");
        telemetry.addData("  Right Stick X", "Rotate");
        telemetry.addData("  L1 (Left Bumper)", "Left Index");
        telemetry.addData("  R1 (Right Bumper)", "Right Index");
        telemetry.addData("  L2 (Left Trigger)", "Intake");
        telemetry.addData("  R2 (Right Trigger)", "Shooter");
        telemetry.update();
    }

    /**
     * Handle mecanum drive calculations and set motor powers
     * Both gamepad1 and gamepad2 can control the robot
     */
    private void handleMecanumDrive() {
        // Combine inputs from both controllers
        double y1 = -gamepad1.left_stick_y;
        double x1 = gamepad1.left_stick_x * STRAFE_MULTIPLIER;
        double rx1 = gamepad1.right_stick_x;
        
        double y2 = -gamepad2.left_stick_y;
        double x2 = gamepad2.left_stick_x * STRAFE_MULTIPLIER;
        double rx2 = gamepad2.right_stick_x;
        
        // Use whichever controller has greater input magnitude
        double y = (Math.abs(y1) > Math.abs(y2)) ? y1 : y2;
        double x = (Math.abs(x1) > Math.abs(x2)) ? x1 : x2;
        double rx = (Math.abs(rx1) > Math.abs(rx2)) ? rx1 : rx2;

        // Calculate motor powers maintaining ratio
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set drive motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Handle intake and shooter motor controls via bumpers and triggers
     * Both gamepad1 and gamepad2 can control accessories
     */
    private void handleAccessoryMotors() {
        // L1 (left bumper) controls left index motor - either controller
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            leftIndexMotor.setPower(1.0);
        } else {
            leftIndexMotor.setPower(0);
        }

        // R1 (right bumper) controls right index motor - either controller
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            rightIndexMotor.setPower(1.0);
        } else {
            rightIndexMotor.setPower(0);
        }

        // L2 (left trigger) controls intake motor - use max of both controllers
        double intakePower = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        intakeMotor.setPower(intakePower);

        // R2 (right trigger) controls shooter motor - use max of both controllers
        double shooterPower = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        shootMotor.setPower(shooterPower);
    }

    /**
     * Update telemetry with current robot status
     */
    private void updateTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("", "");
        telemetry.addData("Drive", "");
        telemetry.addData("  FL Power", "%.2f", frontLeftMotor.getPower());
        telemetry.addData("  FR Power", "%.2f", frontRightMotor.getPower());
        telemetry.addData("  BL Power", "%.2f", backLeftMotor.getPower());
        telemetry.addData("  BR Power", "%.2f", backRightMotor.getPower());
        telemetry.addData("", "");
        telemetry.addData("Accessories", "");
        telemetry.addData("  Left Index", (gamepad1.left_bumper || gamepad2.left_bumper) ? "ON" : "OFF");
        telemetry.addData("  Right Index", (gamepad1.right_bumper || gamepad2.right_bumper) ? "ON" : "OFF");
        telemetry.addData("  Intake", "%.2f", intakeMotor.getPower());
        telemetry.addData("  Shooter", "%.2f", shootMotor.getPower());
        telemetry.update();
    }
}
