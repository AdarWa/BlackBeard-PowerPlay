package org.firstinspires.ftc.teamcode.TeleOpDrive;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpDrive.imu.IMU;

@TeleOp
public class TeleOpDrive extends LinearOpMode {

    enum DriveMode {
        JokerDrive,
        RegularDrive
    }

    private Motor frontLeft, frontRight, backLeft,backRight;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    private DriveMode mode = DriveMode.JokerDrive;

    private double powerMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = new Motor(hardwareMap,"frontLeft");
        frontRight = new Motor(hardwareMap,"frontRight");
        backLeft = new Motor(hardwareMap,"backLeft");
        backRight = new Motor(hardwareMap,"backRight");

        drive = new MecanumDrive(frontLeft,
                                    frontRight,
                                        backLeft,
                                            backRight);
        driverOp = new GamepadEx(gamepad1);


        IMU imu = new IMU(hardwareMap);

        waitForStart();

        double heading = 0;

        while(opModeIsActive()){
            changePowerByRightBumper();

            if(mode == DriveMode.JokerDrive)
                heading = imu.getHeading();

            if(mode == DriveMode.JokerDrive) {
                drive.driveFieldCentric(
                        -driverOp.getLeftX() * powerMultiplier, //strafe
                        -driverOp.getLeftY() * powerMultiplier, //forward
                        -driverOp.getRightX() * powerMultiplier, //turn
                        heading //angle

                );
            }else{
                drive.driveRobotCentric(
                        -driverOp.getLeftX() * powerMultiplier, //strafe
                        -driverOp.getLeftY() * powerMultiplier, //forward
                        -driverOp.getRightX() * powerMultiplier //turn
                );
            }

            telemetry.addData("Heading", heading);
            telemetry.update();
        }

    }

    private void changePowerByRightBumper(){
        powerMultiplier = driverOp.isDown(GamepadKeys.Button.RIGHT_BUMPER) ? 0.5 : 1;
    }

    private void changeModeByLeftBumper(){
        mode = driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ? mode  == DriveMode.JokerDrive ? DriveMode.RegularDrive : DriveMode.JokerDrive : mode;
    }
}