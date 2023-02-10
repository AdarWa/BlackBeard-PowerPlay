package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpDrive.Storage;

@TeleOp(name = "Erase Storage. Be Careful!")
public class EraseStorageOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Warning: this opmode will erase all of the saved positions and headings.", "");
        telemetry.addData("Press Start only if you know what you are doing!", "");
        waitForStart();
        Storage.heading = 0;
        Storage.pose = Storage.startPose;
        while (!isStopRequested() && opModeIsActive());
    }
}
