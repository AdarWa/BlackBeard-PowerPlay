package org.firstinspires.ftc.teamcode.TeleOpDrive;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOpDrive.imu.IMU;

public class Drive {

    enum DriveMode {
        JokerDrive,
        RegularDrive
    }


    private Motor[] motors;
    private MecanumDrive drive;
    private GamepadEx driver;
    private IMU imu;
    private Telemetry telemetry;

    private double heading = 0;

    private DriveMode mode = DriveMode.JokerDrive;

    private double powerMultiplier = 1;

    public Drive(GamepadEx driver,
                 IMU imu,
                 Telemetry telemetry,
                 Motor... motors){
        this.motors = motors;
        drive = new MecanumDrive(motors[0],
                motors[1],
                motors[2],
                motors[3]);

        this.driver = driver;
        this.imu = imu;
        this.telemetry = telemetry;
    }

    public void update(){
        changePowerByRightBumper();
        changeModeByLeftBumper();

        if(mode == DriveMode.JokerDrive)
            heading = imu.getHeading();

        if(mode == DriveMode.JokerDrive) {
            drive.driveFieldCentric(
                    driver.getLeftX() * -powerMultiplier, //strafe
                    driver.getLeftY() * -powerMultiplier, //forward
                    driver.getRightX() * -powerMultiplier, //turn
                    heading //angle

            );
        }else{
            drive.driveRobotCentric(
                    driver.getLeftX() * -powerMultiplier, //strafe
                    driver.getLeftY() * -powerMultiplier, //forward
                    driver.getRightX() * -powerMultiplier //turn
            );
        }

        telemetry.addData("Heading", heading);
        telemetry.update();
    }


    private void changePowerByRightBumper(){
        powerMultiplier = driver.isDown(GamepadKeys.Button.RIGHT_BUMPER) ? 0.5 : 1;
    }

    private void changeModeByLeftBumper(){
        mode = driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ? mode  == DriveMode.JokerDrive ? DriveMode.RegularDrive : DriveMode.JokerDrive : mode;
    }



}
