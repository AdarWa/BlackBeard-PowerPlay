package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private boolean gripperState = false;
    GamepadEx operator;
    Servo servo;

    public Gripper(GamepadEx operator, HardwareMap map){
        this.operator = operator;
        this.servo = map.servo.get("gripper");
    }

    public void update(boolean autoGrip){
//        operator.readButtons();
//        if(operator.wasJustPressed(GamepadKeys.Button.A))
//            gripperState = !gripperState;
        if(!autoGrip){
            servo.setPosition(operator.getButton(GamepadKeys.Button.A) ? 0.8 : 1);
        }else if(operator.getButton(GamepadKeys.Button.A)){
            servo.setPosition(0.8);
        }
    }

    public void ungrip(){
        servo.setPosition(0.8);
    }
    public void grip(){
        servo.setPosition(1);
    }

    public void grip(boolean gripperState){
        if(gripperState){
            grip();
        }else {
            ungrip();
        }
    }

}
