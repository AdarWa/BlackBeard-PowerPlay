package org.firstinspires.ftc.teamcode.vision.aprilTags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Config;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagDetector {

    static class DetectorCalibration{
        public double fx = 578.272;
        public double fy = 578.272;
        public double cx = 402.145;
        public double cy = 221.506;

        // UNITS ARE METERS
        public double tagsize = 0.166;
    }

    public static int[] tags = new int[]{0,1,2};


    public static OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public DetectorCalibration calibration;


    private Telemetry telemetry;


    public AprilTagDetector(HardwareMap hardwareMap, Telemetry telemetry){
        init(hardwareMap, telemetry, new DetectorCalibration());
    }

    public AprilTagDetector(HardwareMap hardwareMap,Telemetry telemetry, DetectorCalibration calibration){
        init(hardwareMap,telemetry, calibration);
    }

    private void init(HardwareMap hardwareMap,Telemetry telemetry, DetectorCalibration calibration){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Config.barcodeCameraName), cameraMonitorViewId);
        this.calibration = calibration;
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(calibration.tagsize, calibration.fx, calibration.fy, calibration.cx, calibration.cy);
        this.telemetry = telemetry;
    }

    public void startCamera(){
        camera.setPipeline(aprilTagDetectionPipeline);
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

        telemetry.setMsTransmissionInterval(50);
    }

    public void closeCamera(OpenCvCamera.AsyncCameraCloseListener listener){
        camera.closeCameraDeviceAsync(listener);
    }

    /**
     * detects the park spot from the sleeve using the AprilTagDetectionPipeline.
     * returns null if no aprilTag was detected.
     * @return the park spot.
     */
    public Integer detect(){
        boolean parkDetected = false;
        int parkSpot = 0;
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {

            for(AprilTagDetection tag : currentDetections)
            {
                for(int i = 0; i < tags.length; i++){
                    if(tag.id == tags[i])
                    {
                        parkSpot = i+1;
                        telemetry.addData("tag"+i, tags[i]);
                        parkDetected = true;
                    }
                }
            }

        }

        return parkDetected ? parkSpot : null;
    }

    public static void setTags(int... t){
        tags = t;
    }

}
