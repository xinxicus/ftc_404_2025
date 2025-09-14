// Autonomous Mecanum Drive OpMode for testing basic movements
// Simple 4-inch movements for all directions - no sensors needed
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class MecanumAutoOp extends LinearOpMode {
    
    // Motor declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    
    // Constants for movement - GoBILDA 5203 motor specifications
    private static final double WHEEL_DIAMETER = 4.0; // inches (adjust based on your wheels)
    private static final double COUNTS_PER_REVOLUTION = 384.5; // GoBILDA 5203 encoder PPR
    private static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER * Math.PI);
    private static final double MOVE_POWER = 0.3; // Movement power
    private static final double TURN_POWER = 0.4; // Turn power
    
    // Simple 4-inch movements for all directions
    private static final double MOVE_DISTANCE = 4.0; // 4 inches for all movements
    private static final double TURN_DEGREES = 90.0; // 90 degrees for turns
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        // Set motor directions (same as TeleOp)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        telemetry.addData("Status", "Starting autonomous sequence");
        telemetry.update();
        
        // Simple autonomous sequence - all movements are 4 inches
        try {
            // 1. Move forward 4 inches
            telemetry.addData("Action", "Moving forward 4 inches");
            telemetry.update();
            moveForwardInches(MOVE_DISTANCE);
            
            // 2. Move backward 4 inches
            telemetry.addData("Action", "Moving backward 4 inches");
            telemetry.update();
            moveBackwardInches(MOVE_DISTANCE);
            
            // 3. Move left 4 inches
            telemetry.addData("Action", "Moving left 4 inches");
            telemetry.update();
            moveLeftInches(MOVE_DISTANCE);
            
            // 4. Move right 4 inches
            telemetry.addData("Action", "Moving right 4 inches");
            telemetry.update();
            moveRightInches(MOVE_DISTANCE);
            
            // 5. Turn left 90 degrees
            telemetry.addData("Action", "Turning left 90 degrees");
            telemetry.update();
            turnLeft(TURN_DEGREES);
            
            // 6. Turn right 90 degrees
            telemetry.addData("Action", "Turning right 90 degrees");
            telemetry.update();
            turnRight(TURN_DEGREES);
            
            telemetry.addData("Status", "Autonomous sequence completed");
            telemetry.update();
            
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Autonomous interrupted");
            telemetry.update();
        }
        
        // Stop all motors
        stopMotors();
    }
    
    // Move forward for specified inches
    private void moveForwardInches(double inches) throws InterruptedException {
        int target = (int) (inches * COUNTS_PER_INCH);
        
        frontLeftMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftMotor.setPower(MOVE_POWER);
        backLeftMotor.setPower(MOVE_POWER);
        frontRightMotor.setPower(MOVE_POWER);
        backRightMotor.setPower(MOVE_POWER);
        
        // Wait until all motors reach target
        while (opModeIsActive() && 
               (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || 
                frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            Thread.sleep(50);
        }
        
        stopMotors();
        resetEncoders();
    }
    
    // Move backward for specified inches
    private void moveBackwardInches(double inches) throws InterruptedException {
        int target = -(int) (inches * COUNTS_PER_INCH);
        
        frontLeftMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftMotor.setPower(MOVE_POWER);
        backLeftMotor.setPower(MOVE_POWER);
        frontRightMotor.setPower(MOVE_POWER);
        backRightMotor.setPower(MOVE_POWER);
        
        // Wait until all motors reach target
        while (opModeIsActive() && 
               (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || 
                frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            Thread.sleep(50);
        }
        
        stopMotors();
        resetEncoders();
    }
    
    // Move left (strafe) for specified inches
    private void moveLeftInches(double inches) throws InterruptedException {
        int target = (int) (inches * COUNTS_PER_INCH);
        
        frontLeftMotor.setTargetPosition(-target);
        backLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(-target);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftMotor.setPower(MOVE_POWER);
        backLeftMotor.setPower(MOVE_POWER);
        frontRightMotor.setPower(MOVE_POWER);
        backRightMotor.setPower(MOVE_POWER);
        
        // Wait until all motors reach target
        while (opModeIsActive() && 
               (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || 
                frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            Thread.sleep(50);
        }
        
        stopMotors();
        resetEncoders();
    }
    
    // Move right (strafe) for specified inches
    private void moveRightInches(double inches) throws InterruptedException {
        int target = (int) (inches * COUNTS_PER_INCH);
        
        frontLeftMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(-target);
        frontRightMotor.setTargetPosition(-target);
        backRightMotor.setTargetPosition(target);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftMotor.setPower(MOVE_POWER);
        backLeftMotor.setPower(MOVE_POWER);
        frontRightMotor.setPower(MOVE_POWER);
        backRightMotor.setPower(MOVE_POWER);
        
        // Wait until all motors reach target
        while (opModeIsActive() && 
               (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || 
                frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            Thread.sleep(50);
        }
        
        stopMotors();
        resetEncoders();
    }
    
    // Turn left for specified degrees
    private void turnLeft(double degrees) throws InterruptedException {
        // Adjust wheelbase based on your robot's actual dimensions
        double wheelbase = 16.0; // inches (adjust based on your robot)
        double arcLength = (degrees * Math.PI * wheelbase) / 360.0;
        int target = (int) (arcLength * COUNTS_PER_INCH);
        
        frontLeftMotor.setTargetPosition(-target);
        backLeftMotor.setTargetPosition(-target);
        frontRightMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftMotor.setPower(TURN_POWER);
        backLeftMotor.setPower(TURN_POWER);
        frontRightMotor.setPower(TURN_POWER);
        backRightMotor.setPower(TURN_POWER);
        
        // Wait until all motors reach target
        while (opModeIsActive() && 
               (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || 
                frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            Thread.sleep(50);
        }
        
        stopMotors();
        resetEncoders();
    }
    
    // Turn right for specified degrees
    private void turnRight(double degrees) throws InterruptedException {
        // Adjust wheelbase based on your robot's actual dimensions
        double wheelbase = 16.0; // inches (adjust based on your robot)
        double arcLength = (degrees * Math.PI * wheelbase) / 360.0;
        int target = (int) (arcLength * COUNTS_PER_INCH);
        
        frontLeftMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(-target);
        backRightMotor.setTargetPosition(-target);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeftMotor.setPower(TURN_POWER);
        backLeftMotor.setPower(TURN_POWER);
        frontRightMotor.setPower(TURN_POWER);
        backRightMotor.setPower(TURN_POWER);
        
        // Wait until all motors reach target
        while (opModeIsActive() && 
               (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || 
                frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            Thread.sleep(50);
        }
        
        stopMotors();
        resetEncoders();
    }
    
    // Stop all motors
    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    
    // Reset encoders
    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
