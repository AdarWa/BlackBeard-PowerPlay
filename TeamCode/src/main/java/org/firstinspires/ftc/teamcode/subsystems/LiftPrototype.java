package org.firstinspires.ftc.teamcode.subsystems;

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
        Low(-3637),
        Mid(-5731);
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
    private PowerPlayOpMode opMode;




    public LiftPrototype(GamepadEx operator, HardwareMap  hardwareMap, PowerPlayOpMode opMode){
        this.operator = operator;
        this.opMode = opMode;
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void controlLift(){
        double power = -operator.getRightY();
        if((power < 0 && liftMotor.getCurrentPosition() < -500) || (power >= 0 && liftMotor.getCurrentPosition() > -5200) || power == 0)
            liftMotor.setPower(-power/(operator.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 1 : 2));
        if(operator.getButton(GamepadKeys.Button.Y)){
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        operator.readButtons();
        if(operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            goToJunc(Junction.Ground);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            goToJunc(Junction.Low);
        else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            goToJunc(Junction.Mid);
        //else if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
        //goToJunc(Junction.High);
    }

    private void goToJunc(Junction junction){
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        debug("JuncState", junction.name());
        liftMotor.setTargetPosition(junction.getTicks());
        liftMotor.setPower(1);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        new Thread(() -> {
            while(liftMotor.isBusy() && !operator.getButton(GamepadKeys.Button.BACK)) {
                opMode.stopAll();

                debug("Pos", String.valueOf(liftMotor.getCurrentPosition()));

            }

            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }).start();
    }

    private void debug(String caption, String data){
        opMode.telemetry.addData(caption,data);
        opMode.telemetry.update();
    }


}
