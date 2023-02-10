package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoBasic;
import org.firstinspires.ftc.teamcode.AutonomousOpMode;

@Autonomous(name = "Autonomous Preload and Park (Right)", group = "Autonomous", preselectTeleOp = "LocalizationTest")
public class AutoPreloadParkRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new AutonomousOpMode(this, AutoBasic.PRELOAD_PARK).runOpMode();
    }
}
