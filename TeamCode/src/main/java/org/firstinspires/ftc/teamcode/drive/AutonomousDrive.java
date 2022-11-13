package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autoPlanB.MotorController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomousDrive {

    /**
     * drive using the RoadRunner library
     */
    public static void drive(RoadRunnerDrive drive, int parkSpot){
        TrajectorySequence sequence = drive.drive.trajectorySequenceBuilder(new Pose2d(-61, -34, 0))
                .lineTo(new Vector2d(-60,-60))
                .splineTo(new Vector2d(0,-60), 0)
                .lineTo(new Vector2d(12,-60))
                .lineTo(new Vector2d(12,0))
                .turn(Math.toRadians(90))
                .build();
        drive.drive.followTrajectorySequence(sequence);
    }

    /**
     * drive without roadrunner
     */
    public static void drive(MotorController controller, int parkSpot){

    }
}
