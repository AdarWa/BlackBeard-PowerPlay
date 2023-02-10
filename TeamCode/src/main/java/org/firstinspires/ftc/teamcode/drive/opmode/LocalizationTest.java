package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeleOpDrive.AllianceColor;
import org.firstinspires.ftc.teamcode.TeleOpDrive.AutoGrip;
import org.firstinspires.ftc.teamcode.TeleOpDrive.Drive;
import org.firstinspires.ftc.teamcode.TeleOpDrive.Storage;
import org.firstinspires.ftc.teamcode.TeleOpDrive.autoLift.AutoLiftController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    private double powerMultiplier = 1;
    public static Telemetry _telemetry = null;
    private static double[] powers = null;
    private boolean cycleMode = false;
    private boolean autoGripEnabled = false;
    private boolean autoLiftEnabled = false;
    private boolean wasFalling = false;

    private void changePowerByRightBumper(){
        if(gamepad1.left_bumper)
            powerMultiplier = 0.9;
        else if(gamepad1.right_bumper)
            powerMultiplier = 0.34;
        else
            powerMultiplier = 0.5;

    }

    public static boolean isMoving(){
        if(powers == null)
            return false;

        for(double power : powers){
            if(power != 0)
                return true;
        }
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        _telemetry = telemetry;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        GamepadEx operator = new GamepadEx(gamepad2);
        LiftPrototype lift = new LiftPrototype(operator, hardwareMap, null);
        Gripper gripper = new Gripper(operator, hardwareMap);
        AutoGrip autoGrip = new AutoGrip(AllianceColor.BLUE, hardwareMap, gripper, telemetry, operator, lift);

        waitForStart();

        drive.setPoseEstimate(Storage.pose);
        LiftPrototype.Junction lastJunc = null;

        while (!isStopRequested()) {
            double fall = getFall(drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            boolean isFallingBack = fall < 0;
            boolean isFalling = isFallingBack ? fall < -5 : fall > 3.5;
            telemetry.addData("fall", fall);
            telemetry.addData("isFalling", isFalling);
            if (isFalling) {
                drive.setMotorPowers(isFallingBack ? 0 : 1, isFallingBack ? -1 : 0, isFallingBack ? -1 : 0, isFallingBack ? 0 : 1);
                wasFalling = true;
                continue;
            } else if (wasFalling) {
                wasFalling = false;
                drive.setMotorPowers(0, 0, 0, 0);
            }

            changePowerByRightBumper();
            double[] powers = Drive.ourShabota(gamepad1.left_stick_x * powerMultiplier, -gamepad1.left_stick_y * powerMultiplier, gamepad1.right_stick_x * powerMultiplier, Math.toDegrees(drive.getExternalHeading())-90 + Storage.heading);
            LocalizationTest.powers = powers;
            drive.setMotorPowers(powers[RobotDrive.MotorType.kFrontLeft.value],
                    powers[RobotDrive.MotorType.kBackLeft.value],
                    powers[RobotDrive.MotorType.kBackRight.value],
                    powers[RobotDrive.MotorType.kFrontRight.value]);

            drive.update();

            operator.readButtons();
            if(operator.wasJustPressed(GamepadKeys.Button.START)){
                cycleMode = !cycleMode;
                operator.gamepad.rumble(700);
            }else if(operator.wasJustPressed(GamepadKeys.Button.Y)){
                autoLiftEnabled = !autoLiftEnabled;
                operator.gamepad.rumble(700);
            }else if(operator.wasJustPressed(GamepadKeys.Button.B)){
                autoGripEnabled = !autoGripEnabled;
                operator.gamepad.rumble(700);
            }


            Pose2d poseEstimate = drive.getPoseEstimate();

            HashMap<Pose2d, LiftPrototype.Junction> map = AutoLiftController.checkPose2d(poseEstimate,cycleMode);

            double heading = Math.toDegrees(drive.getExternalHeading());
            heading = heading > 180 ? heading-360 : heading;

            LiftPrototype.Junction autoLiftJunc = AutoLiftController.checkHeading(
                    map
                    ,poseEstimate, heading, telemetry);

            if(autoLiftJunc != null && autoLiftEnabled){
                if(lastJunc != autoLiftJunc){
                    if(cycleMode){
                        for (Map.Entry<Pose2d, LiftPrototype.Junction> entry : map.entrySet()){
                            lift.goToJunc(entry.getValue());
                            break;
                        }
                    }else {
                        lift.goToJunc(autoLiftJunc);
                        lastJunc = autoLiftJunc;
                    }
                }
            }
            telemetry.addData("lift", autoLiftJunc != null ? autoLiftJunc.toString() : "null");
            telemetry.addData("autoLift", autoLiftEnabled);
            telemetry.addData("autoGrip", autoGripEnabled);
            telemetry.addData("cycleMode", cycleMode);

            telemetry.addData("CurrentPos", lift.liftMotor.getCurrentPosition());
//            telemetry.addData("wheel positions", Arrays.toString(drive.getWheelPositions().toArray()));
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", heading);

//            gripper.update(true);
            if(autoGripEnabled){
                autoGrip.detect();
            }else {
                gripper.update(false);
            }
            lift.controlLift();
        }
        telemetry.update();
    }

    private double getFall(Orientation angles){
        return AngleUnit.DEGREES.normalize(angles.thirdAngle);
    }
}
