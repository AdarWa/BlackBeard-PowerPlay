/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOpDrive.Storage;
import org.firstinspires.ftc.teamcode.TeleOpDrive.imu.IMU;
import org.firstinspires.ftc.teamcode.autoPlanB.MotorController;
import org.firstinspires.ftc.teamcode.drive.AutonomousDrive;
import org.firstinspires.ftc.teamcode.drive.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.LiftPrototype;
import org.firstinspires.ftc.teamcode.vision.aprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.vision.colorDetection.ColorDetector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.AbstractMap;
import java.util.Random;
import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
public class AutonomousOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private RoadRunnerDrive drive;
    private MotorController controller;

    private Integer parkSpot = null; //1 - parkSpot1 2 - parkSpot2 3 - parkSpot3

    private AprilTagDetector aprilDetector;
    private ColorDetector colorDetector;

    private AutoBasic autoType;
    private static LinearOpMode opMode;
    private IMU imu;
    private Gripper gripper;
    private LiftPrototype lift;
    public static Telemetry _telemetry = null;
    private DistanceDetector distanceDetector;

    public static double aprilTagXPose = 0;

    private static final boolean useRoadRunner = true;
    private static final boolean onlyDetect = false;

    public AutonomousOpMode(LinearOpMode opMode, AutoBasic autoType){
        this.opMode = opMode;
        this.autoType = autoType;
    }

    public static LinearOpMode getOpMode(){
        return opMode;
    }

    public static boolean isFinished(){
        return opMode.isStopRequested() || !opMode.opModeIsActive();
    }


    public void runOpMode() {

        _telemetry = opMode.telemetry;
        debug("Status", "Initialized");

        runtime.reset();

        imu = new IMU(opMode.hardwareMap);

        if(useRoadRunner && !onlyDetect){
            drive = new RoadRunnerDrive(opMode.hardwareMap);
            gripper = new Gripper(null, opMode.hardwareMap);
            lift = new LiftPrototype(null,opMode.hardwareMap, null);
            distanceDetector = new DistanceDetector(opMode.hardwareMap, _telemetry, drive.drive);
        }else if(!onlyDetect) {
            controller = new MotorController(
                    new DcMotor[]{
                            opMode.hardwareMap.dcMotor.get("frontLeft"),
                            opMode.hardwareMap.dcMotor.get("frontRight"),
                            opMode.hardwareMap.dcMotor.get("backLeft"),
                            opMode.hardwareMap.dcMotor.get("backRight")
                    }, opMode);
        }
        aprilDetector = new AprilTagDetector(opMode.hardwareMap, opMode.telemetry);
        aprilDetector.startCamera();
        gripper.grip();
//        new Thread(()->{
//            while(true){
////                _telemetry.addData("Req", opMode.isStopRequested());
////                _telemetry.addData("Foo", !opMode.opModeIsActive());
////                _telemetry.update();
//                if(!opMode.opModeIsActive() || opMode.isStopRequested()){
//                    throw new NullPointerException("Foo");
////                    _telemetry.addData("Foo", "Bar");
////                    _telemetry.update();
//                }
//            }
//        }).start();
        recognizeParkSpot();
        opMode.waitForStart();

        if(useRoadRunner && !onlyDetect&&!opMode.isStopRequested() && opMode.opModeIsActive()){
            AutonomousDrive.drive(drive, parkSpot, autoType, gripper, lift,distanceDetector);
        }else if(!onlyDetect){
            AutonomousDrive.drive(controller, parkSpot, autoType);
        }
        Storage.heading = drive.drive.getExternalHeading();
        Storage.pose = drive.drive.getPoseEstimate();
        opMode.stop();
        opMode = null;
    }

    private void recognizeParkSpot(){
        debug("ParkSpotDetection", "AprilTags");
        while(!opMode.isStopRequested()  && !opMode.isStarted()){
            detectAprilTag();
        }
        runtime.reset();
        if(parkSpot == null){
            while (!opMode.isStopRequested() && opMode.opModeIsActive() && (runtime.time(TimeUnit.SECONDS) < 5 )){
                detectAprilTag();
            }
        }
        if(parkSpot == null) {
//            colorDetector.startCamera();
//            telemetry.addData("ParkSpotDetection", "Color");
//            runtime.reset();
//            while (!isStopRequested() && runtime.time(TimeUnit.SECONDS) < 6) {
//                Integer spot = colorDetector.getDetection();
//                if (spot != null) {
//                    parkSpot = spot;
//                    telemetry.addData("ParkSpot", parkSpot);
//                }
//            }
//            if (parkSpot == null) {
                debug("ParkSpotDetection", "Random");
                parkSpot = new Random().nextInt(3) + 1;
                debug("ParkSpot", String.valueOf(parkSpot));
//            }
        }
    }

    private void debug(String caption, String value){
        opMode.telemetry.addData(caption, value);
        opMode.telemetry.update();
    }

    private void detectAprilTag(){
        AbstractMap.SimpleEntry<Integer, AprilTagDetection> entry = aprilDetector.detect();
        Integer spot = entry.getKey();
        AprilTagDetection detection = entry.getValue();
        if(spot != null){
            parkSpot = spot;
            aprilTagXPose = detection.pose.x;
            opMode.telemetry.addData("x", String.valueOf(aprilTagXPose));
            opMode.telemetry.addData("xFix", -((AutonomousOpMode.aprilTagXPose-0.2)*AutonomousDrive.calibrationConst) + 28);
            debug("ParkSpot", String.valueOf(parkSpot));
        }
    }


}
