package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOpDrive.Drive;
import org.firstinspires.ftc.teamcode.TeleOpDrive.imu.IMU;
import org.firstinspires.ftc.teamcode.subsystems.FlipIntake;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;

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



    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = new Motor(hardwareMap,"frontLeft"); //declare the front left motor using the hardware map
            frontRight = new Motor(hardwareMap,"frontRight"); //declare the front right motor using the hardware map
                backLeft = new Motor(hardwareMap,"backLeft"); //declare the back left motor using the hardware map
                    backRight = new Motor(hardwareMap,"backRight"); //declare the back right motor using the hardware map
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

        lift = new LiftPrototype(operator, hardwareMap, this); //initialize the lift



        waitForStart(); //wait until the driver press "Start"


        while(opModeIsActive()){
            drive.update(); //drive using the joystick
//            intake.controlMechanism();
//            flipIntake.intakeControl();
            lift.controlLift(); //control the lift using the joystick
        }

    }
}
