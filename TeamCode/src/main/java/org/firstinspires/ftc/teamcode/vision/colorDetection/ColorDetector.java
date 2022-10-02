package org.firstinspires.ftc.teamcode.vision.colorDetection;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

//TODO: make the actual color detector.
public class ColorDetector {

    private OpenCvCamera camera;
    private ColorDetectionPipeline pipeline;
    private Telemetry telemetry;

    public ColorDetector(HardwareMap hardwareMap, Telemetry telemetry){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorDetectionPipeline();
        this.telemetry = telemetry;
    }

    public void startCamera(){
        camera.setPipeline(pipeline);
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

        telemetry.setMsTransmissionInterval(50);

    }

    public Integer getDetection(){
        return pipeline.detection;
    }


    static class ColorDetectionPipeline extends OpenCvPipeline {

        public Integer detection = null;

        @Override
        public Mat processFrame(Mat input) {
            return input;
        }
    }

}
