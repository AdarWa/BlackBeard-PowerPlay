package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOpDrive.AllianceColor;
import org.firstinspires.ftc.teamcode.TeleOpDrive.AutoGrip;
import org.firstinspires.ftc.teamcode.TeleOpDrive.Drive;
//import org.firstinspires.ftc.teamcode.TeleOpDrive.autoLift.AutoLiftController;
import org.firstinspires.ftc.teamcode.TeleOpDrive.autoLift.AutoLiftController;
import org.firstinspires.ftc.teamcode.TeleOpDrive.imu.IMU;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlipIntake;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;
import org.firstinspires.ftc.teamcode.util.MathEx;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class PowerPlayOpMode extends LinearOpMode {


    private Motor frontLeft, frontRight, backLeft,backRight;
    private Drive drive;
    private GamepadEx driver;
    private GamepadEx operator;

    private Servo intakeServo1;
    private Servo intakeServo2;

    private IntakePrototype1 intake;

    private LiftPrototype lift;

    private SampleMecanumDrive roadrunner;

    private IMU imu;

    public void stopAll(){
        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);
    }



    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap,"frontLeft"); //declare the front left motor using the hardware map
        frontRight = new Motor(hardwareMap,"frontRight"); //declare the front right motor using the hardware map
        backLeft = new Motor(hardwareMap,"backLeft"); //declare the back left motor using the hardware map
        backRight = new Motor(hardwareMap,"backRight"); //declare the back right motor using the hardware map
        


//        roadrunner = new SampleMecanumDrive(hardwareMap, false);
//        roadrunner.setPoseEstimate(new Pose2d(0,0));
//
//        intakeServo1 = hardwareMap.servo.get("intakeServo1");
//        intakeServo2 = hardwareMap.servo.get("intakeServo2");


        driver = new GamepadEx(gamepad1); //declare the driver
        operator = new GamepadEx(gamepad2); //declare the operator
        imu = new IMU(hardwareMap);
        roadrunner = new SampleMecanumDrive(hardwareMap, false, false,
                (DcMotorEx) frontLeft.motor,(DcMotorEx)frontRight.motor,(DcMotorEx)backLeft.motor,(DcMotorEx)backRight.motor);
        drive = new Drive(driver,telemetry,frontLeft,frontRight,backLeft,backRight);
//        roadrunner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//
//        intake = new IntakePrototype1(intakeServo1, intakeServo2, operator);

//        flipIntake = new FlipIntake(operator, hardwareMap);

        lift = new LiftPrototype(operator, hardwareMap, null); //initialize the lift
        Gripper gripper = new Gripper(operator, hardwareMap);
//        AutoGrip autoGrip = new AutoGrip(AllianceColor.RED, hardwareMap, gripper,telemetry);
        waitForStart(); //wait until the driver press "Start"


        while(opModeIsActive()){
            roadrunner.update();
            Pose2d point = roadrunner.getPoseEstimate();
            double heading = Math.toDegrees(point.getHeading());
            telemetry.addData("CurrentPos", lift.liftMotor.getCurrentPosition());
            drive.update(imu.getHeading()); //drive using the joystinck
//            new Thread(lift::controlLift).start();
            lift.controlLift();
            gripper.update(false);
//            autoGrip.detect();
            LiftPrototype.Junction autoLiftJunc = AutoLiftController.checkHeading(
                    AutoLiftController.checkPose2d(point)
                    ,point, heading, telemetry);
//
            telemetry.addData("Lift", autoLiftJunc.toString());
//            telemetry.addData("Heading1", heading);
//            telemetry.addData("x", point.getX());
//            telemetry.addData("y", point.getY());
//            telemetry.update();
            roadrunner.update();
            Pose2d pos = roadrunner.getPoseEstimate();
            telemetry.addData("pos", MathEx.roundOff(pos.getX(),100) + ","+ MathEx.roundOff(pos.getY(),100));
//            telemetry.addData("lift", AutoLiftController.checkPose2d(pos).toString())
//            intake.controlMechanism();
//            flipIntake.intakeControl();
//            lift.controlLift(); //control the lift using the joystick
//            gripper.update();
        }

    }

    public void updateDrive(){
        drive.update(imu.getHeading());
    }
}
