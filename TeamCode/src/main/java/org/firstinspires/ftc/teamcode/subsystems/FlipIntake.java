package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo
        ;

public class FlipIntake {

    private GamepadEx operator;

    private DcMotorEx intakeMotor;
    private Servo flipServo;

    public FlipIntake(GamepadEx operator, HardwareMap hardwareMap){
        this.operator = operator;
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        flipServo = hardwareMap.servo.get("upDownServo");
    }

    public void intakeControl(){
        intakeMotor.setPower(operator.getLeftY());
        if(operator.getButton(GamepadKeys.Button.X)){
            flipServo.setPosition(1);
        }else{
            flipServo.setPosition(0);
        }
    }

    private void sleep(long millis){
        try {Thread.sleep(millis);} catch (InterruptedException e) {e.printStackTrace();}
    }


}
