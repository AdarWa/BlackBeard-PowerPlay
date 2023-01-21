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
    private Telemetry telemetry;

    private double heading = 0;

    private DriveMode mode = DriveMode.JokerDrive;

    private double powerMultiplier = 1;

    public Drive(GamepadEx driver,
                 Telemetry telemetry,
                 Motor... motors){
        this.motors = motors;
        drive = new MecanumDrive(motors[0],
                motors[1],
                motors[2],
                motors[3]);

        this.driver = driver;
        this.telemetry = telemetry;
    }

    public void update(double heading){
        changePowerByRightBumper();
        changeModeByLeftBumper();

//        if(mode == DriveMode.JokerDrive)


        if(mode == DriveMode.JokerDrive) {
            telemetry.addData("Mode", "Joker");
            telemetry.addData("strafe", driver.getLeftX());
            telemetry.addData("forward", driver.getLeftY());
            telemetry.addData("turn", driver.getRightX());
            telemetry.addData("powerMultiplier", -powerMultiplier);
            telemetry.addData("heading", HeadingStorage.heading);
            telemetry.update();
            drive.driveFieldCentric(
                    driver.getLeftX() * -powerMultiplier, //strafe
                    driver.getLeftY() * -powerMultiplier, //forward
                    driver.getRightX() * -powerMultiplier, //turn
                    heading + HeadingStorage.heading //angle

            );
        }else{
            drive.driveRobotCentric(
                    driver.getLeftX() * -powerMultiplier, //strafe
                    driver.getLeftY() * -powerMultiplier, //forward
                    driver.getRightX() * -powerMultiplier //turn
            );
        }

//        telemetry.addData("Heading", heading);
//        telemetry.update();
    }


    private void changePowerByRightBumper(){
        if(driver.isDown(GamepadKeys.Button.LEFT_BUMPER))
            powerMultiplier = 0.9;
        else if(driver.isDown(GamepadKeys.Button.RIGHT_BUMPER))
            powerMultiplier = 0.39;
        else
            powerMultiplier = 0.5;

    }

    private void changeModeByLeftBumper(){
        mode = driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ? mode  == DriveMode.JokerDrive ? DriveMode.RegularDrive : DriveMode.JokerDrive : mode;
    }



}
