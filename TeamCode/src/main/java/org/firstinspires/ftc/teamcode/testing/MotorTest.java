package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor1 = hardwareMap.dcMotor.get("motor1");
        DcMotor motor2 = hardwareMap.dcMotor.get("motor2");
        waitForStart();
        while (opModeIsActive()){
            motor1.setPower(gamepad1.right_stick_y);
            motor2.setPower(gamepad1.right_stick_y);
        }
    }
}
