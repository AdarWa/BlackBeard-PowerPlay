package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    private GamepadEx operator;

    private DcMotorEx intakeMotor;
    private Servo upDownServo;

    public Intake(GamepadEx operator, HardwareMap hardwareMap){
        this.operator = operator;
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        upDownServo = hardwareMap.servo.get("upDownServo");
    }

    public void intakeControl(){
        intakeMotor.setPower(operator.getLeftY());
        if(intakeMotor.getCurrent(CurrentUnit.MILLIAMPS) >= 600){
            sleep(300);
            upDownServo.setPosition(1);
            sleep(700);
            upDownServo.setPosition(0);
        }
    }

    private void sleep(long millis){
        try {Thread.sleep(millis);} catch (InterruptedException e) {e.printStackTrace();}
    }


}
