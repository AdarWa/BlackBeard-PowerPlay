package org.firstinspires.ftc.teamcode.TeleOpDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Storage {
    public static final Pose2d startPose = new Pose2d(40, -64,Math.toRadians(90));

    public static double heading = 0;
    public static Pose2d pose = startPose;
}
