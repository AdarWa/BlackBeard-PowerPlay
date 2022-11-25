package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    GamepadEx operator;
    Servo servo;

    public Gripper(GamepadEx operator, HardwareMap map){
        this.operator = operator;
        this.servo = map.servo.get("gripper");
    }

    public void update(){
        servo.setPosition(operator.getButton(GamepadKeys.Button.A) ? 0 : 1/3);
    }

}
