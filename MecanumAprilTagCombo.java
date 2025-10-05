package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class MecanumAprilTagCombo extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    //private DcMotor shootMotor;
    //private CRServo intakeServo;

    private double y = 0;
    private double x = 0;
    private double rx = 0;
    final double DESIRED_DISTANCE = 48.0; 
    final double SPEED_GAIN =   0.035 ;   
    final double TURN_GAIN  =   0.017;   

    final double MAX_AUTO_SPEED = 0.75;   
    final double MAX_AUTO_TURN  = 0.25;  
    private static final boolean USE_WEBCAM = true;  
    private static final int DESIRED_TAG_ID = 24;    
    private VisionPortal visionPortal;               
    private AprilTagProcessor aprilTag;              
    private AprilTagDetection desiredTag = null;     

    @Override public void runOpMode()
    {
        boolean targetFound     = false;    
        double  drive           = 0;        
        double  turn            = 0;        

        
        initAprilTag();

        
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        //shootMotor = hardwareMap.dcMotor.get("shootMotor");
        //intakeServo = hardwareMap.crservo.get("intakeServo");

        //If motor ever acts up, disable frontLeftMotor and backLeftMotor .setDircetion() methods 
        // and enable the frontRightMotor and backRightMotor ones instead.
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // We might need one, all, or none of the below.
        // shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);


        if (USE_WEBCAM)
            setManualExposure(6, 250);  
        
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

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
            
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }
            

            if (gamepad1.left_bumper) {
                if (targetFound) {
                    double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  yawError = (desiredTag.ftcPose.yaw);
                    double  headingError = desiredTag.ftcPose.bearing;
                    
                    y = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    x = Range.clip(yawError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    rx  = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                    telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", y, rx);
                } else {
                    y = 0;
                    x = 0;
                    rx  = 0;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                    telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", y, rx);
                }

            } else {
                
                y = -gamepad1.left_stick_y;  
                x = gamepad1.left_stick_x * 1.1; // to counteract imperfect strafing;
                rx  = gamepad1.right_stick_x;  
                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }
            telemetry.update();




            // --- Intake and Shooter Control ---
            // The triggers return a value from 0.0 to 1.0, which is perfect for continuous power
            //double intakePower = gamepad1.left_trigger;
            // double shootPower = gamepad1.right_trigger;

            // Set power to the intake servo and shooter motor
            //  intakeServo.setPower(intakePower);
            //  shootMotor.setPower(shootPower);

            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Yaw is counter-clockwise
     */

//    public void moveRobot(double y, double x, double rx) {
//        // Calculate left and right wheel powers.
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;



    // Send powers to the wheels.
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

//}

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
