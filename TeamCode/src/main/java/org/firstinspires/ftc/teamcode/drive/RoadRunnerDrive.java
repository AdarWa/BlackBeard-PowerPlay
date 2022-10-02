package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RoadRunnerDrive {

    public SampleMecanumDrive drive;
    private Pose2d lastPosition = startPosition;
    private static final Pose2d startPosition = new Pose2d(0,0,0);

    public RoadRunnerDrive(HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
    }

    public Pose2d getLastPosition(){
        return lastPosition;
    }

    public Trajectory lineTo(Vector2d pos){
        return drive.trajectoryBuilder(lastPosition)
                .lineTo(pos)
                .build();
    }

    public Trajectory splineTo(Vector2d pos, double heading){
        return drive.trajectoryBuilder(lastPosition)
                .splineTo(pos,heading)
                .build();
    }
}
