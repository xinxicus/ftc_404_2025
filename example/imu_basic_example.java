package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * BNO055IMU Basic Usage Example
 * 
 * This is a simplified example showing the essential steps for using the IMU:
 * 1. Declaration and initialization
 * 2. Reading basic orientation data
 * 3. Displaying data on telemetry
 */
@TeleOp(name = "IMU Basic Example")
public class IMUBasicExample extends LinearOpMode {
    
    // IMU sensor declaration
    private BNO055IMU imu;
    
    @Override
    public void runOpMode() {
        // Step 1: Get IMU from hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        // Step 2: Create and configure IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;  // Use degrees instead of radians
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        
        // Step 3: Initialize IMU with parameters
        imu.initialize(parameters);
        
        telemetry.addData("Status", "IMU Initialized - Press Start");
        telemetry.update();
        
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            // Step 4: Read orientation data from IMU
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            
            // Step 5: Extract individual angles
            double heading = angles.firstAngle;   // Z-axis (heading/yaw) - robot rotation
            double pitch = angles.secondAngle;    // Y-axis (pitch) - forward/backward tilt
            double roll = angles.thirdAngle;      // X-axis (roll) - left/right tilt
            
            // Step 6: Display data on driver station
            telemetry.addData("Heading (Z)", "%.1f degrees", heading);
            telemetry.addData("Pitch (Y)", "%.1f degrees", pitch);
            telemetry.addData("Roll (X)", "%.1f degrees", roll);
            
            // Additional useful information
            telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "Yes" : "No");
            telemetry.addData("System Calibrated", imu.isSystemCalibrated() ? "Yes" : "No");
            
            telemetry.addLine();
            telemetry.addLine("Axis Explanation:");
            telemetry.addLine("• Heading (Z): Robot rotation (most important for navigation)");
            telemetry.addLine("• Pitch (Y): Forward/backward tilt");
            telemetry.addLine("• Roll (X): Left/right tilt");
            
            telemetry.update();
            
            // Small delay to make telemetry readable
            sleep(100);
        }
    }
}
