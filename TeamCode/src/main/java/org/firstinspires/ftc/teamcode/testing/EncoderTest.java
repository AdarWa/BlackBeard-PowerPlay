package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "testing")
public class EncoderTest extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor dcMotor = hardwareMap.dcMotor.get("backLeft");
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive()){
            dcMotor.setPower(1);
            dcMotor.setTargetPosition(10000);
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (dcMotor.isBusy() & opModeIsActive()){
                telemetry.addData("encoderPosition", dcMotor.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}
