package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AutoBasic;
import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autoPlanB.MotorController;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class AutonomousDrive {

    /**
     * drive using the RoadRunner library
     */
    public static void drive(RoadRunnerDrive drive, int parkSpot, AutoBasic autoType, Gripper gripper, LiftPrototype lift){
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
        driveByParkSpot(drive, parkSpot, autoType, gripper, lift);
//        if (trajectory != null)
//            drive.drive.followTrajectorySequence(trajectory);
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

    private static void driveByParkSpot(RoadRunnerDrive drive, int parkSpot, AutoBasic autoType, Gripper gripper, LiftPrototype lift){
        int multiplier = autoType == AutoBasic.TO_RIGHT ? 1 : -1;
        Trajectory trajectory = drive.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(Utils.cmToInch(22))
                        .build();
        drive.drive.followTrajectory(trajectory);
        trajectory = drive.drive.trajectoryBuilder(new Pose2d())
                .forward(Utils.tileToInch(2.5)).build();
        drive.drive.followTrajectory(trajectory);
        drive.drive.turn(Math.toRadians(53));
//        trajectory = drive.drive.trajectoryBuilder(new Pose2d())
//                .forward(Utils.cmToInch(20)).build();
//        drive.drive.followTrajectory(trajectory);
        lift.goToJunc(LiftPrototype.Junction.High);
        drive.drive.setExternalHeading(0);
        trajectory = drive.drive.trajectoryBuilder(trajectory.end(),Math.toRadians(0))
                .forward(Utils.cmToInch(40))
                .build();
        drive.drive.followTrajectory(trajectory);
        lift.goToJunc(LiftPrototype.Junction.Mid);
        gripper.ungrip();
//        TrajectorySequenceBuilder sqeuence = drive.drive.trajectorySequenceBuilder(new Pose2d());
//
//
//        sqeuence = sqeuence
//                .strafeLeft(Utils.cmToInch(22))
//                .forward(Utils.tileToInch(2.5))
//                .turn(Math.toRadians(53))
//                .addDisplacementMarker(500,()->{
//                    AutonomousOpMode._telemetry.addData("current", TrajectorySequenceBuilder.currentDisplacement);
//                    AutonomousOpMode._telemetry.update();

//                });
//                .turn(Math.toRadians(-53))
//                .strafeRight(Utils.cmToInch(24));

//        if(parkSpot != 2)
//            sqeuence = sqeuence.back(Utils.cmToInch(25) + Utils.tileToInch(1.5));
//        if(parkSpot == 2){
//            sqeuence = sqeuence
//                    .forward(Utils.cmToInch(20));
//        }
//        else if(parkSpot == 1) {
//            sqeuence = sqeuence
//                    .strafeLeft(Utils.tileToInch(1.35))
//                    .forward(Utils.tileToInch(1.7));
//        }
//        else  if(parkSpot == 3) {
//            sqeuence = sqeuence
//                    .strafeRight(Utils.tileToInch(1.35))
//                    .forward(Utils.tileToInch(1.7));
//        }
//        return sqeuence.build();
    }
}
