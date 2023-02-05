package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.vision.aprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.vision.colorDetection.ColorDetector;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class VisionTest extends LinearOpMode {

    class Pipeline extends OpenCvPipeline {

        public PoleDetection poleDetection = null;
        public Pipeline(){
            poleDetection = new PoleDetection();
        }


        @Override
        public Mat processFrame(Mat input) {
            return poleDetection.process(input, telemetry, gamepad1);
        }
    }

    public static OpenCvCamera camera;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Config.gripperCameraName), cameraMonitorViewId);
        Gripper gripper = new Gripper(new GamepadEx(gamepad1), hardwareMap);
        waitForStart();
        camera.setPipeline(new Pipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error!", errorCode);
            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        while (opModeIsActive() && !isStopRequested()){
            gripper.update(false);
            Point point =  PoleDetection.detectedCircle;
            telemetry.addData("centerPoint",point != null ? point.x + "," + point.y : "null");
            telemetry.update();
        }
        camera.closeCameraDevice();
    }
}
