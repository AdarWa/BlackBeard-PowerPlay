package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor[] motors = new DcMotor[]{
                hardwareMap.dcMotor.get("frontRight"),
                hardwareMap.dcMotor.get("backRight"),
                hardwareMap.dcMotor.get("frontLeft"),
                hardwareMap.dcMotor.get("backLeft")
        };
        waitForStart();
        while (opModeIsActive()){
            for(DcMotor motor : motors){
                motor.setPower(1);
                Thread.sleep(5000);
                motor.setPower(0);
            }
            
        }
    }
}
