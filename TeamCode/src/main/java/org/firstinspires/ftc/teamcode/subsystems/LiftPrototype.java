package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftPrototype {

    public enum Junction{
        Ground(478),
        Low(1870);
//        Mid(-1108),
//        High(0);

        private int i;

        Junction(int i) {
            this.i = i;
        }

        int getTicks(){
            return i;
        }
    }


    public DcMotor liftMotor;
    private GamepadEx operator;
    private LinearOpMode opMode;




    public LiftPrototype(GamepadEx operator, HardwareMap  hardwareMap, LinearOpMode opMode){
        this.operator = operator;
        this.opMode = opMode;
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void controlLift(){
        liftMotor.setPower(operator.getRightY()/2);
        operator.readButtons();
        if(operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            goToJunc(Junction.Ground);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            goToJunc(Junction.Low);
//        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
//            goToJunc(Junction.Mid);
        //else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
        //goToJunc(Junction.High);
    }

    private void goToJunc(Junction junction){
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        debug("JuncState", junction.name());
        liftMotor.setTargetPosition(junction.getTicks());
        liftMotor.setPower(1);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(liftMotor.isBusy() && !operator.getButton(GamepadKeys.Button.BACK)) {


            debug("Pos", String.valueOf(liftMotor.getCurrentPosition()));

        }

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void debug(String caption, String data){
        opMode.telemetry.addData(caption,data);
        opMode.telemetry.update();
    }


}
