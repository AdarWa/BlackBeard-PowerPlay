package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftPrototype {

    enum Junction{
        Ground(0),
        Low(0),
        Mid(0),
        High(0);

        private int i;

        Junction(int i) {
            this.i = i;
        }

        int getTicks(){
            return i;
        }
    }


    private DcMotor liftMotor;
    private GamepadEx operator;
    private LinearOpMode opMode;

    public LiftPrototype(GamepadEx operator, HardwareMap  hardwareMap, LinearOpMode opMode){
        this.operator = operator;
        this.liftMotor = hardwareMap.dcMotor.get("liftMotor");
        this.opMode = opMode;
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void controlLift(){
        liftMotor.setPower(operator.getRightY()/4);

        if(operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            goToJunc(Junction.Ground);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            goToJunc(Junction.Low);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            goToJunc(Junction.Mid);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            goToJunc(Junction.High);

    }

    private void goToJunc(Junction junction){
        liftMotor.setTargetPosition(junction.getTicks());
        liftMotor.setPower(1);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(liftMotor.isBusy())
            opMode.idle();

        liftMotor.setPower(0);
    }


}
