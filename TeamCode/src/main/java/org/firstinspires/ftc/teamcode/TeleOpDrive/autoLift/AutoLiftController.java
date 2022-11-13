package org.firstinspires.ftc.teamcode.TeleOpDrive.autoLift;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;

import java.util.HashMap;
import java.util.Map;

public class AutoLiftController {

    private static HashMap<Pose2d, LiftPrototype.Junction> junctions = new HashMap<>();
    private static final double proximityConst = 15;

    private AutoLiftController(){

    }

    static {
        initJunctions();
    }


    private static void initJunctions(){
        junctions.put(new Pose2d(0,47.5), LiftPrototype.Junction.Ground);
        junctions.put(new Pose2d(47.5,47.5), LiftPrototype.Junction.Ground);
        junctions.put(new Pose2d(-47.5,0), LiftPrototype.Junction.Ground);
        junctions.put(new Pose2d(0,0), LiftPrototype.Junction.Ground);
        junctions.put(new Pose2d(47.5,0), LiftPrototype.Junction.Ground);
        junctions.put(new Pose2d(-47.5,-47.5), LiftPrototype.Junction.Ground);
        junctions.put(new Pose2d(0,-47.5), LiftPrototype.Junction.Ground);
        junctions.put(new Pose2d(47.5,-47.5), LiftPrototype.Junction.Ground);

        junctions.put(new Pose2d(-23.5,47.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(23.5,47.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(-47.5,23.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(47.5,23.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(-47.5,23.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(47.5,-23.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(-23.5,-47.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(23.5,-47.5), LiftPrototype.Junction.Low);

        junctions.put(new Pose2d(-23.5,-23.5), LiftPrototype.Junction.Mid);
        junctions.put(new Pose2d(23.5,23.5), LiftPrototype.Junction.Mid);
        junctions.put(new Pose2d(23.5,-23.5), LiftPrototype.Junction.Mid);
        junctions.put(new Pose2d(-23.5,-23.5), LiftPrototype.Junction.Mid);

        junctions.put(new Pose2d(0,-23.5), LiftPrototype.Junction.High);
        junctions.put(new Pose2d(0,23.5), LiftPrototype.Junction.High);
        junctions.put(new Pose2d(23.5,0), LiftPrototype.Junction.High);
        junctions.put(new Pose2d(-23.5,0), LiftPrototype.Junction.High);
    }



    public static LiftPrototype.Junction checkPose2d(Pose2d currentPos){
        for (Map.Entry<Pose2d, LiftPrototype.Junction> entry : junctions.entrySet()){
            Pose2d Pose2d = entry.getKey();
            LiftPrototype.Junction junc = entry.getValue();
            if(Math.abs(Math.abs(Pose2d.getX()) - Math.abs(currentPos.getX())) <= proximityConst && Math.abs(Math.abs(Pose2d.getY()) - Math.abs(currentPos.getY())) <= proximityConst)
                return junc;
        }
        return null;
    }

}
