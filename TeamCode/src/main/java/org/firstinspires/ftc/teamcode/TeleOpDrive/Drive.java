package org.firstinspires.ftc.teamcode.TeleOpDrive;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
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

    public static final double kDefaultRangeMin = -1.0;
    public static final double kDefaultRangeMax = 1.0;
    public static final double kDefaultMaxSpeed = 1.0;

    protected static double rangeMin = kDefaultRangeMin;
    protected static double rangeMax = kDefaultRangeMax;
    protected static double maxOutput = kDefaultMaxSpeed;

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

    public static double[] ourShabota(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle){
        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] += turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] -= turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] += turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] -= turnSpeed;

        normalize(wheelSpeeds);
        return wheelSpeeds;
    }

    public static double clipRange(double value) {
        return value <= rangeMin ? rangeMin
                : value >= rangeMax ? rangeMax
                : value;
    }

    protected static void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    /**
     * Normalize the wheel speeds
     */
    protected static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }



    private void changePowerByRightBumper(){
        if(driver.isDown(GamepadKeys.Button.LEFT_BUMPER))
            powerMultiplier = 0.9;
        else if(driver.isDown(GamepadKeys.Button.RIGHT_BUMPER))
            powerMultiplier = 0.34;
        else
            powerMultiplier = 0.5;

    }

    private void changeModeByLeftBumper(){
        mode = driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ? mode  == DriveMode.JokerDrive ? DriveMode.RegularDrive : DriveMode.JokerDrive : mode;
    }



}
