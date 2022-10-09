package org.firstinspires.ftc.teamcode.vision.colorDetection;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.firstinspires.ftc.teamcode.vision.aprilTags.AprilTagDetector.camera;

public class ColorDetector {

    private ColorDetectionPipeline pipeline;
    private Telemetry telemetry;

    public ColorDetector(HardwareMap hardwareMap, Telemetry telemetry){
        pipeline = new ColorDetectionPipeline();
        this.telemetry = telemetry;
    }

    public void startCamera(){
        camera.setPipeline(pipeline);
    }


    public Integer getDetection(){
        return pipeline.detection;
    }


    static class ColorDetectionPipeline extends OpenCvPipeline {

        public Integer detection = null;

        // TOPLEFT anchor point for the bounding box
        private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

        // Width and height for the bounding box
        public static int REGION_WIDTH = 30;
        public static int REGION_HEIGHT = 50;

        // Lower and upper boundaries for colors
        private static final Scalar
                lower_yellow_bounds  = new Scalar(200, 200, 0, 255),
                upper_yellow_bounds  = new Scalar(255, 255, 130, 255),
                lower_cyan_bounds    = new Scalar(0, 200, 200, 255),
                upper_cyan_bounds    = new Scalar(150, 255, 255, 255),
                lower_magenta_bounds = new Scalar(170, 0, 170, 255),
                upper_magenta_bounds = new Scalar(255, 60, 255, 255);

        // Color definitions
        private final Scalar
                YELLOW  = new Scalar(255, 255, 0),
                CYAN    = new Scalar(0, 255, 255),
                MAGENTA = new Scalar(255, 0, 255);

        // Percent and mat definitions
        private double yelPercent, cyaPercent, magPercent;
        private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), blurredMat = new Mat();

        // Anchor point definitions
        Point sleeve_pointA = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_pointB = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Running variable storing the parking position

        @Override
        public Mat processFrame(Mat input) {
            // Noise reduction
            Imgproc.blur(input, blurredMat, new Size(5, 5));
            blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

            // Apply Morphology
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

            // Gets channels from given source mat
            Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
            Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
            Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

            // Gets color specific values
            yelPercent = Core.countNonZero(yelMat);
            cyaPercent = Core.countNonZero(cyaMat);
            magPercent = Core.countNonZero(magMat);

            // Calculates the highest amount of pixels being covered on each side
            double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent));

            // Checks all percentages, will highlight bounding box in camera preview
            // based on what color is being detected
            if (maxPercent == yelPercent) {
                detection = 1;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        YELLOW,
                        2
                );
            } else if (maxPercent == cyaPercent) {
                detection = 2;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        CYAN,
                        2
                );
            } else if (maxPercent == magPercent) {
                detection = 3;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        MAGENTA,
                        2
                );
            }

            // Memory cleanup
            blurredMat.release();
            yelMat.release();
            cyaMat.release();
            magMat.release();
            return input;
        }
    }

}
