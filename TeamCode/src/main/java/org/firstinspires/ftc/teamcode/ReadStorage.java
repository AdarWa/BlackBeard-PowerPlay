package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpDrive.Storage;

@TeleOp(name = "ReadStorage")
public class ReadStorage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d pose = Storage.pose;
        telemetry.addData("Pose", "(x,y)("+pose.getX() + "," + pose.getY() +")");
        telemetry.addData("heading", Storage.heading + "°");
        telemetry.update();
        while (!isStopRequested());
    }
}
