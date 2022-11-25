package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOpDrive.Drive;
import org.firstinspires.ftc.teamcode.TeleOpDrive.autoLift.AutoLiftController;
import org.firstinspires.ftc.teamcode.TeleOpDrive.imu.IMU;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlipIntake;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;
import org.firstinspires.ftc.teamcode.util.MathEx;

@TeleOp
public class PowerPlayOpMode extends LinearOpMode {


    private Motor frontLeft, frontRight, backLeft,backRight;
    private Drive drive;
    private GamepadEx driver;
    private GamepadEx operator;

    private Servo intakeServo1;
    private Servo intakeServo2;

    private IntakePrototype1 intake;

    private FlipIntake flipIntake;

    private LiftPrototype lift;

    private SampleMecanumDrive roadrunner;



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


        IMU imu = new IMU(hardwareMap); //initialize the IMU

        drive = new Drive(driver,imu,telemetry,frontLeft,frontRight,backLeft,backRight);
//
//        intake = new IntakePrototype1(intakeServo1, intakeServo2, operator);

//        flipIntake = new FlipIntake(operator, hardwareMap);

//        lift = new LiftPrototype(operator, hardwareMap, this); //initialize the lift
        Gripper gripper = new Gripper(operator, hardwareMap);
        waitForStart(); //wait until the driver press "Start"


        while(opModeIsActive()){
//            roadrunner.update();
//            Pose2d pos = roadrunner.getPoseEstimate();
//            telemetry.addData("pos", MathEx.roundOff(pos.getX(),100) + ","+ MathEx.roundOff(pos.getY(),100));
//            telemetry.addData("lift", AutoLiftController.checkPose2d(pos).toString());
            drive.update(); //drive using the joystick
//            intake.controlMechanism();
//            flipIntake.intakeControl();
//            lift.controlLift(); //control the lift using the joystick
            gripper.update();
        }

    }
}
