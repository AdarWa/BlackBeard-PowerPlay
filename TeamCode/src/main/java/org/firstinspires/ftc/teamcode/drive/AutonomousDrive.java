package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AutoBasic;
import org.firstinspires.ftc.teamcode.autoPlanB.MotorController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

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
        int multiplier = autoType == AutoBasic.TO_RIGHT ? 1 : -1;
        if(parkSpot == 2){
            controller.forward(0.8, 0.5);
        }else if(parkSpot == 1){
            controller.strafeLeft(1.2*multiplier, 0.3);
            controller.forward(0.8, 0.5);
        }else if(parkSpot == 3){
            controller.strafeRight(1.2*multiplier, 0.3);
            controller.forward(0.8, 0.5);
        }
    }

    private static TrajectorySequence driveByParkSpot(RoadRunnerDrive drive, int parkSpot, AutoBasic autoType){
        int multiplier = autoType == AutoBasic.TO_RIGHT ? 1 : -1;
        TrajectorySequenceBuilder sqeuence = drive.drive.trajectorySequenceBuilder(new Pose2d(-61, -34 * multiplier));

//        sqeuence = sqeuence.strafeRight(Utils.cmToInch(52))
//                .forward(Utils.cmToInch(137))
//                .strafeLeft(Utils.cmToInch(85)).waitSeconds(4);

        if(parkSpot == 2){
            sqeuence = sqeuence
                    .forward(Utils.cmToInch(1));
        }
        else if(parkSpot == 1) {
            sqeuence = sqeuence.strafeLeft(Utils.cmToInch(30));
        }
        else  if(parkSpot == 3) {
            sqeuence = sqeuence.strafeRight(Utils.cmToInch(90));
        }
        return sqeuence.build();
    }
}
