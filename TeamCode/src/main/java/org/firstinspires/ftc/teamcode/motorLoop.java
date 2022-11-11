package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "testing")
public class motorLoop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");

        waitForStart();
        while (opModeIsActive()){
            liftMotor.setPower(-gamepad2.right_stick_y);
            if(gamepad2.right_bumper)
                liftMotor.setPower(liftMotor.getPower()/2);

            telemetry.addData("power", liftMotor.getPower());
            telemetry.update();
        }
    }
}
