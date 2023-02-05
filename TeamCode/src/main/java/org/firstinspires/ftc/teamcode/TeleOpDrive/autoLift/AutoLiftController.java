package org.firstinspires.ftc.teamcode.TeleOpDrive.autoLift;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class AutoLiftController {

    private static HashMap<Pose2d, LiftPrototype.Junction> junctions = new HashMap<>();
    private static final double proximityConst = 20;

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
        junctions.put(new Pose2d(0.8,-60), LiftPrototype.Junction.Ground); //human player
        junctions.put(new Pose2d(0,-52), LiftPrototype.Junction.Ground); //human player
        junctions.put(new Pose2d(-0.8,-60), LiftPrototype.Junction.Ground); //human player

        junctions.put(new Pose2d(-23.5,47.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(23.5,47.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(-47.5,23.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(47.5,23.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(47.5,-23.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(-23.5,-47.5), LiftPrototype.Junction.Low);
        junctions.put(new Pose2d(23.5,-47.5), LiftPrototype.Junction.Low);

        junctions.put(new Pose2d(-23.5,-23.5), LiftPrototype.Junction.Mid);
        junctions.put(new Pose2d(23.5,23.5), LiftPrototype.Junction.Mid);
        junctions.put(new Pose2d(23.5,-23.5), LiftPrototype.Junction.Mid);
        junctions.put(new Pose2d(-23.5,-23.5), LiftPrototype.Junction.Mid);
        junctions.put(new Pose2d(-23.5,23.5), LiftPrototype.Junction.Mid);

        junctions.put(new Pose2d(0,-23.5), LiftPrototype.Junction.High);
        junctions.put(new Pose2d(0,23.5), LiftPrototype.Junction.High);
        junctions.put(new Pose2d(23.5,0), LiftPrototype.Junction.High);
        junctions.put(new Pose2d(-23.5,0), LiftPrototype.Junction.High);

        ArrayList<Pose2d> seenBefore = new ArrayList<>();
        for(Map.Entry<Pose2d, LiftPrototype.Junction> entry : junctions.entrySet()){
            Pose2d point = entry.getKey();
            if(seenBeforeContains(point, seenBefore)){
                System.err.println("Duplicates found! (" + point.getX() + "," + point.getY() + ")");
            }
        }
    }

    private static boolean seenBeforeContains(Pose2d point, ArrayList<Pose2d> array){
        for(Pose2d point1 : array){
            if(point1.getX() == point.getX() && point1.getY() == point.getY()){
                return true;
            }
        }
        return false;
    }


    public static HashMap<Pose2d,LiftPrototype.Junction> checkPose2d(Pose2d currentPos){
        return  checkPose2d(currentPos,false);
    }

    public static HashMap<Pose2d,LiftPrototype.Junction> checkPose2d(Pose2d currentPos, boolean cycleMode){
        HashMap<Pose2d, LiftPrototype.Junction> junctionClose = new HashMap();
        for (Map.Entry<Pose2d, LiftPrototype.Junction> entry : junctions.entrySet()){
            Pose2d point = entry.getKey();
            LiftPrototype.Junction junc = entry.getValue();
            if(junc != LiftPrototype.Junction.High && junc != LiftPrototype.Junction.Ground && cycleMode)
                continue;
            if(point.getX() == 0 && point.getY() == -47.5 && cycleMode)
                continue;
            double distance = Math.sqrt(Math.pow(currentPos.getX() - point.getX(), 2) + Math.pow(currentPos.getY() - point.getY(), 2));
            if(distance <= (cycleMode ? proximityConst - 10 : proximityConst)) {
                junctionClose.put(point,junc);
//                System.out.println(point.x + " " + point.y + " : " + Math.floor(distance) + " " + junc.toString());
            }
        }
        return junctionClose;
    }

    public static LiftPrototype.Junction checkHeading(HashMap<Pose2d,LiftPrototype.Junction> juncs, Pose2d currentPosition, double heading , Telemetry telemetry){
//        heading += 180;
//        heading = Range.scale(heading,0,360, -180,180);
        HashMap<Pose2d,Double> dTans = new HashMap<>();
        for(Map.Entry<Pose2d,LiftPrototype.Junction> entry : juncs.entrySet()){
            Pose2d point = entry.getKey();
            LiftPrototype.Junction junc = entry.getValue();

            double y = point.getY() - currentPosition.getY();
            double x = point.getX() - currentPosition.getX();
            double atan = Math.atan2(y,x);
            dTans.put(point,Math.abs(Math.toRadians(heading) - atan));
        }
        double min = Double.MAX_VALUE;
        Pose2d minPose2d = null;
        for(Map.Entry<Pose2d, Double> entry : dTans.entrySet()){
            double tan = entry.getValue();
            if(tan < min){
                min = tan;
                minPose2d = entry.getKey();
            }
        }
//        System.out.println(minPose2d);
        return juncs.get(minPose2d);
    }

}
