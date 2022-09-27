package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakePrototype1 {

    private static final GamepadKeys.Button intakeButton = GamepadKeys.Button.A;

    private Servo servo1;
    private Servo servo2;

    private GamepadEx operator;

    private boolean servoState;

    public IntakePrototype1(Servo servo1, Servo servo2, GamepadEx operator) {
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.operator = operator;
    }

    public void controlMechanism() {
        if (operator.wasJustPressed(intakeButton)) {
            servo1.setPosition(servoState ? 0 : 1);
            servo2.setPosition(servoState ? 0 : 1);
            servoState = !servoState;
        }
    }
}


