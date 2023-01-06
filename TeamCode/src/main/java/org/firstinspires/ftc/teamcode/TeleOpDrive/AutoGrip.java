package org.firstinspires.ftc.teamcode.TeleOpDrive;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class AutoGrip {

    private AllianceColor alliance;
    private HardwareMap hardwareMap;
    private Rev2mDistanceSensor distance;
    private ColorSensor color;
    private Gripper gripper;
    private Telemetry telemetry;

    public AutoGrip(AllianceColor alliance, HardwareMap hardwareMap, Gripper gripper, Telemetry telemetry){
        this.alliance = alliance;
        this.hardwareMap = hardwareMap;
        distance = (Rev2mDistanceSensor)(hardwareMap.get(DistanceSensor.class, "distance"));
        color = hardwareMap.colorSensor.get("color");
        this.gripper = gripper;
        this.telemetry = telemetry;
    }

    public void detect(){
        telemetry.addData("color RGB:", color.red() + " " + color.green() + " " + color.blue());
        telemetry.addData("distance", distance.getDistance(DistanceUnit.MM));
        if(isRed(color.red(),color.blue(), color.green()) &&
                distance.getDistance(DistanceUnit.MM) < 155 &&
                distance.getDistance(DistanceUnit.MM) > 28 &&
                alliance == AllianceColor.RED){
            gripper.grip();
        }else{
            gripper.ungrip();
        }
    }

    public boolean isRed(int r,int g,int b){
        return (r > 100 && g < r/2 && b < r/2);
    }

}
