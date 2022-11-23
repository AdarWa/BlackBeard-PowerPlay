package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoBasic;
import org.firstinspires.ftc.teamcode.AutonomousOpMode;

@Autonomous(name = "Autonomous To Left", group = "Autonomous", preselectTeleOp = "PowerPlayOpMode")
public class AutoToLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new AutonomousOpMode(this, AutoBasic.TO_LEFT).runOpMode();
    }
}
