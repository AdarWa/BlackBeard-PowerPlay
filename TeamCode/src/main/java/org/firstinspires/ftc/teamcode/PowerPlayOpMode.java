package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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

        frontLeft = new Motor(hardwareMap,"frontLeft");
            frontRight = new Motor(hardwareMap,"frontRight");
                backLeft = new Motor(hardwareMap,"backLeft");
                    backRight = new Motor(hardwareMap,"backRight");

        intakeServo1 = hardwareMap.servo.get("intakeServo1");
        intakeServo2 = hardwareMap.servo.get("intakeServo2");


        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);


        IMU imu = new IMU(hardwareMap);

        drive = new Drive(driver,imu,telemetry,frontLeft,frontRight,backLeft,backRight);

        intake = new IntakePrototype1(intakeServo1, intakeServo2, operator);

        flipIntake = new FlipIntake(operator, hardwareMap);

        lift = new LiftPrototype(operator, hardwareMap, this);



        waitForStart();


        while(opModeIsActive()){
            drive.update();
            intake.controlMechanism();
            flipIntake.intakeControl();
            lift.controlLift();
        }

    }
}
