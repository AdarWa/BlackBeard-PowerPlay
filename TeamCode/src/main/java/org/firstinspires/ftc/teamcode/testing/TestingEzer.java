package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(group = "testing")
public class TestingEzer extends LinearOpMode {
    private DcMotor saveMotor;
    @Override
    public void runOpMode() throws InterruptedException{
        saveMotor = hardwareMap.get(DcMotor.class, "SaveMotor");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                saveMotor.setTargetPosition(500);
                saveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                saveMotor.setPower(1);
                while (saveMotor.isBusy()&&opModeIsActive()){

                    }

                }

            }
        }
    }

