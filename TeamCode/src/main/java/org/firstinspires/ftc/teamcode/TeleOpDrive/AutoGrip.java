package org.firstinspires.ftc.teamcode.TeleOpDrive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;

public class AutoGrip {

    private AllianceColor alliance;
    private Rev2mDistanceSensor distance;
    private Gripper gripper;
    private Telemetry telemetry;
    private GamepadEx operator;
    private  LiftPrototype lift = null;
    private boolean gripperState = false;
    private boolean isLiftInQue = false;
    private double counter = 0;

    public AutoGrip(AllianceColor alliance, HardwareMap hardwareMap, Gripper gripper, Telemetry telemetry, GamepadEx operator, LiftPrototype lift){
        this.alliance = alliance;
        distance = (Rev2mDistanceSensor)(hardwareMap.get(DistanceSensor.class, "distance"));
        this.gripper = gripper;
        this.telemetry = telemetry;
        this.operator = operator;
        this.lift = lift;
    }

    public void detect() throws InterruptedException {
        double d = distance.getDistance(DistanceUnit.MM);
        boolean detected = d <= 60;
        if(detected)
            counter++;
        if(detected != gripperState && lift != null){
            isLiftInQue = true;
            counter = 0;
        }
        if(isLiftInQue && LocalizationTest.isMoving() && counter >= 300){
            lift.goToJunc(LiftPrototype.Junction.AutoGrip);
            isLiftInQue = false;
        }
        gripper.grip(operator.getButton(GamepadKeys.Button.A) ? false : detected);
        telemetry.addData("detectedSomethingUsingDistance",detected);
        telemetry.addData("distance", d);
        telemetry.update();
        gripperState = detected;
    }

}
