package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpDrive.Drive;
import org.firstinspires.ftc.teamcode.TeleOpDrive.imu.IMU;

@TeleOp
public class PowerPlayOpMode extends LinearOpMode {


    private Motor frontLeft, frontRight, backLeft,backRight;
    private Drive drive;
    private GamepadEx driver;



    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = new Motor(hardwareMap,"frontLeft");
        frontRight = new Motor(hardwareMap,"frontRight");
        backLeft = new Motor(hardwareMap,"backLeft");
        backRight = new Motor(hardwareMap,"backRight");


        driver = new GamepadEx(gamepad1);


        IMU imu = new IMU(hardwareMap);

        drive = new Drive(driver,imu,telemetry,frontLeft,frontRight,backLeft,backRight);

        waitForStart();


        while(opModeIsActive()){
            drive.update();
        }

    }
}
