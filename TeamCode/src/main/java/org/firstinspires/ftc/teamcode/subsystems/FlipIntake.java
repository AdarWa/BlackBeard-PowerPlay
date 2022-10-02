package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlipIntake {

    private GamepadEx operator;

    private DcMotor rightMotor;
    private DcMotor leftMotor;

    public FlipIntake(GamepadEx operator, HardwareMap hardwareMap){
        this.operator = operator;
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
    }

    public void intakeControl(){
        rightMotor.setPower(-operator.getLeftY());
        leftMotor.setPower(operator.getLeftY());
    }


    private void sleep(long millis){
        try {Thread.sleep(millis);} catch (InterruptedException e) {e.printStackTrace();}
    }


}
