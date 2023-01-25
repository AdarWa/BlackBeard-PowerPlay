package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutonomousOpMode;
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

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class VisionTest extends LinearOpMode {

    static class Pipeline extends OpenCvPipeline {

        private Mat blurOutput = new Mat();
        private Mat hslThresholdOutput = new Mat();
        private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
        private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
        private Mat source;

        static {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        }

        /**
         * This is the primary method that runs the entire pipeline and updates the outputs.
         */

        public Mat matOfPointToMat(ArrayList<MatOfPoint> matOfPoints){
//		Mat mat = Mat.zeros(source.size(), CV_8UC1);
//		Imgproc.circle (

//				mat,                 //Matrix obj of the image
//				point,    //Center of the circle
//				2,                    //Radius
//				new Scalar(0, 0, 255),  //Scalar object for color
//				10                      //Thickness of the circle
//		);
            Mat mat = source;
            for(MatOfPoint contour : matOfPoints){
                Point[] points = contour.toArray();
                for(Point point : points){
//				 System.out.println(point);
                    Imgproc.circle (
                            mat,                 //Matrix obj of the image
                            point,    //Center of the circle
                            2,                    //Radius
                            new Scalar(0, 0, 255),  //Scalar object for color
                            10                      //Thickness of the circle
                    );
                }
            }
            return mat;
        }

        /**
         * This method is a generated getter for the output of a Blur.
         * @return Mat output from Blur.
         */
        public Mat blurOutput() {
            return blurOutput;
        }

        /**
         * This method is a generated getter for the output of a HSL_Threshold.
         * @return Mat output from HSL_Threshold.
         */
        public Mat hslThresholdOutput() {
            return hslThresholdOutput;
        }

        /**
         * This method is a generated getter for the output of a Find_Contours.
         * @return ArrayList<MatOfPoint> output from Find_Contours.
         */
        public ArrayList<MatOfPoint> findContoursOutput() {
            return findContoursOutput;
        }

        /**
         * This method is a generated getter for the output of a Filter_Contours.
         * @return ArrayList<MatOfPoint> output from Filter_Contours.
         */
        public ArrayList<MatOfPoint> filterContoursOutput() {
            return filterContoursOutput;
        }


        /**
         * An indication of which type of filter to use for a blur.
         * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
         */
        enum BlurType{
            BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
            BILATERAL("Bilateral Filter");

            private final String label;

            BlurType(String label) {
                this.label = label;
            }

            public static BlurType get(String type) {
                if (BILATERAL.label.equals(type)) {
                    return BILATERAL;
                }
                else if (GAUSSIAN.label.equals(type)) {
                    return GAUSSIAN;
                }
                else if (MEDIAN.label.equals(type)) {
                    return MEDIAN;
                }
                else {
                    return BOX;
                }
            }

            @Override
            public String toString() {
                return this.label;
            }
        }

        /**
         * Softens an image using one of several filters.
         * @param input The image on which to perform the blur.
         * @param type The blurType to perform.
         * @param doubleRadius The radius for the blur.
         * @param output The image in which to store the output.
         */
        private void blur(Mat input, BlurType type, double doubleRadius,
                          Mat output) {
            int radius = (int)(doubleRadius + 0.5);
            int kernelSize;
            switch(type){
                case BOX:
                    kernelSize = 2 * radius + 1;
                    Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                    break;
                case GAUSSIAN:
                    kernelSize = 6 * radius + 1;
                    Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                    break;
                case MEDIAN:
                    kernelSize = 2 * radius + 1;
                    Imgproc.medianBlur(input, output, kernelSize);
                    break;
                case BILATERAL:
                    Imgproc.bilateralFilter(input, output, -1, radius, radius);
                    break;
            }
        }

        /**
         * Segment an image based on hue, saturation, and luminance ranges.
         *
         * @param input The image on which to perform the HSL threshold.
         * @param hue The min and max hue
         * @param sat The min and max saturation
         * @param lum The min and max luminance
         */
        private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
                                  Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
            Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
                    new Scalar(hue[1], lum[1], sat[1]), out);
        }

        /**
         * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
         * @param input The image on which to perform the Distance Transform.
         */
        private void findContours(Mat input, boolean externalOnly,
                                  List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            }
            else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);
        }


        /**
         * Filters out contours that do not meet certain criteria.
         * @param inputContours is the input list of contours
         * @param output is the the output list of contours
         * @param minArea is the minimum area of a contour that will be kept
         * @param minPerimeter is the minimum perimeter of a contour that will be kept
         * @param minWidth minimum width of a contour
         * @param maxWidth maximum width
         * @param minHeight minimum height
         * @param maxHeight maximimum height
         * @param minVertexCount minimum vertex Count of the contours
         * @param maxVertexCount maximum vertex Count
         * @param minRatio minimum ratio of width to height
         * @param maxRatio maximum ratio of width to height
         */
        private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                    double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                            maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                            minRatio, double maxRatio, List<MatOfPoint> output) {
            final MatOfInt hull = new MatOfInt();
            output.clear();
            //operation
            for (int i = 0; i < inputContours.size(); i++) {
                final MatOfPoint contour = inputContours.get(i);
                final Rect bb = Imgproc.boundingRect(contour);
                if (bb.width < minWidth || bb.width > maxWidth) continue;
                if (bb.height < minHeight || bb.height > maxHeight) continue;
                final double area = Imgproc.contourArea(contour);
                if (area < minArea) continue;
                if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++) {
                    int index = (int)hull.get(j, 0)[0];
                    double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                final double solid = 100 * area / Imgproc.contourArea(mopHull);
                if (solid < solidity[0] || solid > solidity[1]) continue;
                if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
                final double ratio = bb.width / (double)bb.height;
                if (ratio < minRatio || ratio > maxRatio) continue;
                output.add(contour);
            }
        }

        @Override
        public Mat processFrame(Mat source0) {
            source = source0;
            Mat blurInput = source0;
            BlurType blurType = BlurType.get("Box Blur");
            double blurRadius = 7.207207207207207;
            blur(blurInput, blurType, blurRadius, blurOutput);

            // Step HSL_Threshold0:
            Mat hslThresholdInput = blurOutput;
            double[] hslThresholdHue = {30.84892086330935, 146.21160409556313};
            double[] hslThresholdSaturation = {60.50179856115108, 144.0358361774744};
            double[] hslThresholdLuminance = {36.690647482014384, 185.3754266211604};
            hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);

            // Step Find_Contours0:
            Mat findContoursInput = hslThresholdOutput;
            boolean findContoursExternalOnly = false;
            findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

            // Step Filter_Contours0:
            ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
            double filterContoursMinArea = 1600.0;
            double filterContoursMinPerimeter = 0.0;
            double filterContoursMinWidth = 0;
            double filterContoursMaxWidth = 1000;
            double filterContoursMinHeight = 0;
            double filterContoursMaxHeight = 1000;
            double[] filterContoursSolidity = {43.16546762589928, 100};
            double filterContoursMaxVertices = 1000000;
            double filterContoursMinVertices = 35.0;
            double filterContoursMinRatio = 0;
            double filterContoursMaxRatio = 1000;
            filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
            return matOfPointToMat(filterContoursOutput);
        }
    }

    public static OpenCvCamera camera;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
        while (opModeIsActive() && !isStopRequested());
    }
}
