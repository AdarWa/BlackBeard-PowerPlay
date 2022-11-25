package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AutoBasic;
import org.firstinspires.ftc.teamcode.autoPlanB.MotorController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomousDrive {

    /**
     * drive using the RoadRunner library
     */
    public static void drive(RoadRunnerDrive drive, int parkSpot, AutoBasic autoType){
//        TrajectorySequence sequence = drive.drive.trajectorySequenceBuilder(new Pose2d(-61, -34, 0))
//                .lineTo(new Vector2d(-60,-60))
//                .splineTo(new Vector2d(0,-60), 0)
//                .lineTo(new Vector2d(12,-60))
//                .lineTo(new Vector2d(12,0))
//                .turn(Math.toRadians(90))
//                .build();
//        drive.drive.followTrajectorySequence(sequence);
//        TrajectorySequence trajectory = drive.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
//                .lineTo(new Vector2d(0,50))
//                .build();
        TrajectorySequence trajectory = driveByParkSpot(drive, parkSpot, autoType);
        if (trajectory != null)
            drive.drive.followTrajectorySequence(trajectory);
    }

    /**
     * drive without roadrunner
     */
    public static void drive(MotorController controller, int parkSpot, AutoBasic autoType){

    }

    private static TrajectorySequence driveByParkSpot(RoadRunnerDrive drive, int parkSpot, AutoBasic autoType){
        int multiplier = autoType == AutoBasic.TO_RIGHT ? 1 : -1;
        if(parkSpot == 3){
            return drive.drive.trajectorySequenceBuilder(new Pose2d(-61, -34 * multiplier , 0))
                    .lineTo(new Vector2d(-60, -66 * multiplier))
                    .lineTo(new Vector2d(-29, -66 * multiplier))
                    .build();
        }else if(parkSpot == 2){
            return drive.drive.trajectorySequenceBuilder(new Pose2d(-61, -34 * multiplier, 0))
                    .lineTo(new Vector2d(-60, -70 * multiplier))
                    .lineTo(new Vector2d(5, -70 * multiplier))
                    .turn(Math.toRadians(90 * multiplier))
                    .lineTo(new Vector2d(8, -42 * multiplier))
                    .build();
        }else if(parkSpot == 1){
            return drive.drive.trajectorySequenceBuilder(new Pose2d(-61, -34 * multiplier, 0))
                    .lineTo(new Vector2d(-60, -70 * multiplier))
                    .lineTo(new Vector2d(5, -70 * multiplier))
                    .turn(Math.toRadians(90 * multiplier))
                    .lineTo(new Vector2d(8, -8 * multiplier))
                    .turn(Math.toRadians(90 * multiplier))
                    .lineTo(new Vector2d(-20, -8 * multiplier))
                    .build();
        }
        return null;
    }
}
