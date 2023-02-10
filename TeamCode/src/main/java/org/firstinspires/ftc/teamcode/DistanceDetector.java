package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DistanceDetector {

    private Rev2mDistanceSensor distanceSensor;
    private Telemetry telemetry;
    private SampleMecanumDrive drive;
    public static double distance = 0;
    public  static int counter = 0;
    public static boolean flag = true;
    public DistanceDetector(HardwareMap hardwareMap, Telemetry telemetry, SampleMecanumDrive drive){
        distanceSensor = (Rev2mDistanceSensor)(hardwareMap.get(DistanceSensor.class, "distanceLift"));
        this.telemetry = telemetry;
        this.drive = drive;

    }
    public void powerAll(double power){
        for(DcMotorEx motor : drive.getMotors()){
            motor.setPower(power);
        }
    }

    public void printTelemetry(){
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        telemetry.addData("distance", distance);
        telemetry.update();
    }


    public void distanceFromPole() {
        distance = distanceSensor.getDistance(DistanceUnit.MM);
//        telemetry.addData("distance", distance);
//        telemetry.update();
        if (distance >= 349) {
            drive.turn(Math.toRadians(-2));
            counter++;
//            double power = (Kp * (distance - 70));
//            powerAll(power);
//            counter = 0;
//
//            if (Math.abs(Math.abs(distance) - 140) <= 6) {
//                powerAll(0);
//                flag = false;
//            }
        }


    }

}
