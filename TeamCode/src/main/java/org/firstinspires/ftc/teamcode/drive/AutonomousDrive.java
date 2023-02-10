package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoBasic;
import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.DistanceDetector;
import org.firstinspires.ftc.teamcode.TeleOpDrive.Storage;
import org.firstinspires.ftc.teamcode.autoPlanB.MotorController;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
public class AutonomousDrive {

    public static double calibrationConst = 5;

    /**
     * drive using the RoadRunner library
     */
    public static void drive(RoadRunnerDrive drive, int parkSpot, AutoBasic autoType, Gripper gripper, LiftPrototype lift,DistanceDetector distanceDetector){
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
        driveByParkSpot(distanceDetector,drive, parkSpot, autoType, gripper, lift);
//        if (trajectory != null)
//            drive.drive.followTrajectorySequence(trajectory);
    }

    /**
     * drive without roadrunner
     */
    public static void drive(MotorController controller, int parkSpot, AutoBasic autoType){
//        int multiplier = autoType == AutoBasic.TO_RIGHT ? 1 : -1;
//        if(parkSpot == 2){
//            controller.forward(0.8, 0.5);
//        }else if(parkSpot == 1){
//            controller.strafeLeft(1.2*multiplier, 0.3);
//            controller.forward(0.8, 0.5);
//        }else if(parkSpot == 3){
//            controller.strafeRight(1.2*multiplier, 0.3);
//            controller.forward(0.8, 0.5);
//        }
    }

    private static void driveByParkSpot(DistanceDetector distanceDetector, RoadRunnerDrive drive, int parkSpot, AutoBasic autoType, Gripper gripper, LiftPrototype lift){
        if(autoType == AutoBasic.PARK){

            TrajectorySequenceBuilder sequence;
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
            sequence = drive.drive.trajectorySequenceBuilder(new Pose2d());
            TrajectorySequence s1 = null;

            if(parkSpot == 2){
                s1 = sequence.forward(Utils.tileToInch(1.5))
                        .build();
            }else if(parkSpot == 1){
                s1 = sequence.forward(Utils.tileToInch(1))
                        .strafeRight(Utils.tileToInch(1))
                        .build();
            }else if(parkSpot == 3){
                s1 = sequence.forward(Utils.tileToInch(1))
                        .strafeLeft(Utils.tileToInch(1))
                        .build();
            }
            drive.drive.followTrajectorySequence(s1);
            return;
        }
        new Thread(()->{
            while (AutonomousOpMode.getOpMode().opModeIsActive()){
                distanceDetector.printTelemetry();
            }
        }).start();
        int multiplier = autoType == AutoBasic.TO_RIGHT ? 1 : -1;
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()){
        new Thread(()->lift.goToJunc(LiftPrototype.Junction.Low, gripper)).start();
        gripper.grip();
//        }
        TrajectorySequence s1;
        TrajectorySequenceBuilder sequence;
        //- left
        //+ right
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
        drive.drive.setPoseEstimate(Storage.startPose);
        sequence = drive.drive.trajectorySequenceBuilder(Storage.startPose);
        s1 = sequence.strafeLeft(Utils.cmToInch(-((AutonomousOpMode.aprilTagXPose-0.2)*calibrationConst) + 28)* multiplier)
                    .forward(Utils.tileToInch(0.5))
//                .waitSeconds(0.1)
                .forward(Utils.tileToInch(0.5))
//                .waitSeconds(0.1)
                .forward(Utils.tileToInch(0.5))
//                .waitSeconds(0.1)
                .forward(Utils.tileToInch(0.5))
//                .waitSeconds(0.1)
                .forward(Utils.tileToInch(0.5))
                .forward(Utils.tileToInch(0.3))
                .back(Utils.tileToInch(0.3))
//              .forward(Utils.tileToInch(0.5))
//                .forward(Utils.tileToInch(0.5))

//                .back(Utils.tileToInchwq(0.5))
//                .back(Utils.tileToInch(0.5))

//                .waitSeconds(0.3)
                    //move 2.5 tiles
                .turn(Math.toRadians(37* multiplier))
                .build();
//        }
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
            drive.drive.followTrajectorySequence(s1);
//        }
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();
        while (DistanceDetector.flag && runtime.time() < 2.5 &&
                AutonomousOpMode.getOpMode().opModeIsActive() &&
                !AutonomousOpMode.getOpMode().isStopRequested()){
            distanceDetector.printTelemetry();
            distanceDetector.distanceFromPole();
//            drive.drive.update();
        }

            lift.goToJunc(LiftPrototype.Junction.High, gripper);
            Pose2d pose = drive.drive.getPoseEstimate();
            sequence = drive.drive.trajectorySequenceBuilder(new Pose2d(pose.getX(), pose.getY(), drive.drive.getExternalHeading()));
            gripper.grip();
//        }
//        gripper.grip();

        TrajectorySequence s2 = null;
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
        s2 = sequence.forward(Utils.cmToInch(DistanceDetector.distance <= 310 ? Math.abs((DistanceDetector.distance/10)) : 30))
                .waitSeconds(1.5).build();
        drive.drive.followTrajectorySequence(s2);
//        }
//        lift.goToJunc(LiftPrototype.Junction.Low, gripper);
        gripper.ungrip();
        //go to stack

        sequence = drive.drive.trajectorySequenceBuilder(s2.end());

        TrajectorySequence s3 = null;
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
        s3 = sequence
                .back(Utils.cmToInch(DistanceDetector.distance <= 310 ? Math.abs((DistanceDetector.distance/10 - 10)) : 20)).build();

        drive.drive.followTrajectorySequence(s3);
//        }
//
//
//
//        lift.goToJunc(LiftPrototype.Junction.Low);
//        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
//
//            new Thread(() -> lift.goToJunc(LiftPrototype.Junction.Stack1)).start();
//        }
//        sequence = drive.drive.trajectorySequenceBuilder(s3.end());
//        TrajectorySequence s4 = sequence
//                .turn(Math.toRadians(-60* multiplier))
//                .back(Utils.tileToInch(0.2))
//                .turn(Math.toRadians(-90* multiplier))
//                .forward(Utils.tileToInch(1.2) - Utils.cmToInch(10))
//                .build();
////        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
//            drive.drive.followTrajectorySequence(s4);
////        }
////
////
////        sequence = drive.drive.trajectorySequenceBuilder(s4.end());
////        TrajectorySequence s5 = sequence
////                .forward(Utils.tileToInch(1.2)+ Utils.cmToInch(2))
////                .build();
////        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
////
////            drive.drive.followTrajectorySequence(s5);
////        }
//        gripper.grip();
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        lift.goToJunc(LiftPrototype.Junction.Low);
////
//        sequence = drive.drive.trajectorySequenceBuilder(s4.end());
//        TrajectorySequence s6 = sequence
//                .back(Utils.tileToInch(0.3) - Utils.cmToInch(6))
//                .turn(Math.toRadians(-129))
//                .forward(Utils.tileToInch(0.2) - Utils.cmToInch(2.5))
//                .build();
////        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
////
////        }
////        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
////
//            drive.drive.followTrajectorySequence(s6);
////        }
////        new Thread(()->{
////        }).start();
////
//        gripper.ungrip();
//
////        sequence = drive.drive.trajectorySequenceBuilder(s6.end());
////
////        TrajectorySequence s7 = sequence
////                .back(Utils.tileToInch(0.2))
////                .turn(Math.toRadians(129))
////                .back(Utils.tileToInch(1.2))
////                .strafeRight(Utils.tileToInch(0.7))
////                .build();
//////        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
////
////            drive.drive.followTrajectorySequence(s7);
////        }
////
////        sequence = drive.drive.trajectorySequenceBuilder(s7.end());
////
////        TrajectorySequence s8 = sequence
////                .back(Utils.cmToInch(17))
////                .build();
////
////        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
////
////            drive.drive.followTrajectorySequence(s8);
////        }
////        if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {
////
////        }

        if(autoType == AutoBasic.PRELOAD_PARK){
            new Thread(() -> lift.goToJunc(LiftPrototype.Junction.Low)).start();

            sequence = drive.drive.trajectorySequenceBuilder(s3.end());
            if(parkSpot == 2){
                TrajectorySequence s9 = sequence
                        .turn(Math.toRadians(DistanceDetector.counter*2+39))
                        .back(Utils.tileToInch(0.3))
    //                    .turn(Math.toRadians(129))
    //                    .back(Utils.tileToInch(0.9))
    //                    .strafeRight(Utils.tileToInch(1.2))
                        .build();
    //            if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {

                    drive.drive.followTrajectorySequence(s9);
    //            }
            }else if(parkSpot == 3){
                TrajectorySequence s9 = sequence
                        .turn(Math.toRadians(DistanceDetector.counter*2+39))
    //                    .back(Utils.tileToInch(0.35))
    //                    .turn(Math.toRadians(129))
    //                    .back(Utils.tileToInch(1.9))
    //                    .strafeRight(Utils.tileToInch(1.2))
                        .back(Utils.tileToInch(1.25))
                        .build();
    //            if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {

                    drive.drive.followTrajectorySequence(s9);
    //            }
            }else if(parkSpot == 1){
                TrajectorySequence s9 = sequence
    ////                    .turn(Math.toRadians(-37))
    //////                    .back(Utils.tileToInch(0.3))
    ////                    .strafeRight(Utils.tileToInch(1.2))
    ////                    .back(Utils.tileToInch(1))
    //                    .back(Utils.tileToInch(0.2))
    //                    .turn(Math.toRadians(129))
    ////                    .back(Utils.tileToInch(1.2))
    //                    .strafeRight(Utils.tileToInch(0.7))
                        .turn(Math.toRadians(DistanceDetector.counter*2+39))
    //                    .back(Utils.tileToInch(0.35))
    //                    .strafeLeft(Utils.tileToInch(1.2))
                        .forward(Utils.tileToInch(1))
                        .strafeLeft(Utils.tileToInch(0.6))
                        .build();
    //            if(!AutonomousOpMode.getOpMode().isStopRequested() && AutonomousOpMode.getOpMode().opModeIsActive()) {

                    drive.drive.followTrajectorySequence(s9);
    //            }
            }
        }





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
