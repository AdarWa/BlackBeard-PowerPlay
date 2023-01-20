package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PowerPlayOpMode;

public class LiftPrototype {

    public enum Junction{
        Ground(0),
        Low(1069),
        Mid(1757),
        High(2494),
        Stack1(356);

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
    private PowerPlayOpMode opMode;
    private volatile boolean threadInt = false;



    public LiftPrototype(GamepadEx operator, HardwareMap  hardwareMap, PowerPlayOpMode opMode){
        this.operator = operator;
        this.opMode = opMode;
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void controlLift(){
//        if(!liftMotor.isBusy()){
//            liftMotor.setPower(0);
//            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        if(threadInt){
//            threadInt = false;
//            stopAfterEncoders();
//        }
        double power = -operator.getLeftY();
//        if((power < 0 && liftMotor.getCurrentPosition() < 0) || (power >= 0 && liftMotor.getCurrentPosition() > -5700) || power == 0)
            liftMotor.setPower(-power/(operator.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 2 : 1));
//        if(operator.getButton(GamepadKeys.Button.Y)){
//        if (liftMotor.setMode(DcMotor.DPAD_RIGHT))
        if(operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            goToJunc(Junction.Ground);
        if(operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            goToJunc(Junction.Low);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            goToJunc(Junction.Mid);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            goToJunc(Junction.High);
    }

    public void goToJunc(Junction junction, Gripper gripper){
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        debug("JuncState", junction.name());
        liftMotor.setTargetPosition(junction.getTicks());
        liftMotor.setPower(1);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        new Thread(() -> {
        while (liftMotor.isBusy()) {
//            if(opMode != null)
//                opMode.updateDrive();
            if(gripper != null){
                gripper.grip();
            }
            debug("Pos", String.valueOf(liftMotor.getCurrentPosition()));
            if(operator != null){
                if(operator.getButton(GamepadKeys.Button.BACK)){
                    break;
                }
            }
        }
        stopAfterEncoders();
//            threadInt = true;
//        }).start();
    }
    private void stopAfterEncoders(){
        debug("Debug", "Debug");

        liftMotor.setPower(0);
        Log.e("Lift", "Got Here!");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goToJunc(Junction junction){
        goToJunc(junction, null);
    }

    private void debug(String caption, String data){
        if(opMode != null){
            opMode.telemetry.addData(caption,data);
            opMode.telemetry.update();
        }
    }


}
