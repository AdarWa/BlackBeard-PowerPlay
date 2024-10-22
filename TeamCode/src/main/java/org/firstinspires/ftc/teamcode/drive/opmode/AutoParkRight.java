package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoBasic;
import org.firstinspires.ftc.teamcode.AutonomousOpMode;

@Autonomous(name = "Autonomous Park", group = "Autonomous", preselectTeleOp = "PowerPlayOpMode")
public class AutoPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new AutonomousOpMode(this, AutoBasic.PARK).runOpMode();
    }
}