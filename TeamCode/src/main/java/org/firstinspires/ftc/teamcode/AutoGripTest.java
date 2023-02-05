package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpDrive.AllianceColor;
import org.firstinspires.ftc.teamcode.TeleOpDrive.AutoGrip;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@TeleOp
public class AutoGripTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gripper gripper = new Gripper(null, hardwareMap);
        AutoGrip autoGrip = new AutoGrip(AllianceColor.BLUE, hardwareMap, gripper, telemetry, new GamepadEx(gamepad2), null);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            autoGrip.detect();
        }
    }
}
