package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Teleop Test")
public class TeleopTest extends OpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor1;
    private DcMotor armMotor2;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Robot Status", "Robot is ready");
    }

    @Override
    public void loop() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        if (gamepad2.a) {
            armMotor1.setPower(1);
        } else if (gamepad2.b) {
            armMotor1.setPower(-1);
        } else {
            armMotor1.setPower(0);
        }

        if (gamepad2.x) {
            armMotor2.setPower(1);
        } else if (gamepad2.y) {
            armMotor2.setPower(-1);
        } else {
            armMotor2.setPower(0);
        }

        leftDrive.setPower(leftPower);
        //leftDrive2.setPower(leftPower);
        rightDrive.setPower(rightPower);
        //leftDrive2.setPower(leftPower);
    }
}

