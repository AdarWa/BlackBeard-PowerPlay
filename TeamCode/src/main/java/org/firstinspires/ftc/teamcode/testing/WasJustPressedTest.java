package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "testing")
public class WasJustPressedTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepad = new GamepadEx(gamepad2);
        waitForStart();
        int counter = 0;
        while (opModeIsActive()){
            gamepad.readButtons();
            if(gamepad.wasJustPressed(GamepadKeys.Button.A)){
                counter++;
            }
            telemetry.addData("counter", counter);
            telemetry.addData("state", gamepad.getButton(GamepadKeys.Button.A));
            telemetry.update();
//            Thread.sleep(1000);
        }
    }
}
