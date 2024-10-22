package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PoleDetection {

    //Outputs
//    private Mat blurOutput = new Mat();
//    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private Mat source0Copy = null;
    double[] hsvThresholdValue = {0.0 , 109.80546075085323};
//    private Mat blurInput;
//    private Mat hsvThresholdInput;
//    private Mat findContoursInput;
//    private Mat hierarchy;

    public static Point detectedCircle = null;

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public Mat process(Mat source0, Telemetry telemetry, Gamepad gamepad) {
        source0Copy = source0.clone();
        // Step Blur0:
        BlurType blurType = BlurType.BOX;
        double blurRadius = 31.53153153153153;
        blur(source0, blurType, blurRadius, source0);

//        // Step HSV_Threshold0:
//        hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {0.0, 180.0};
        double[] hsvThresholdSaturation = {0.0, 255.0};
        hsvThresholdValue[0] += gamepad.a ? 1 : gamepad.b ? -1 : 0;
        hsvThresholdValue[1] += gamepad.x ? 1 : gamepad.y ? -1 : 0;
        telemetry.addData("ValueMin", hsvThresholdValue[0]);
        telemetry.addData("ValueMax", hsvThresholdValue[1]);
        telemetry.update();
        hsvThreshold(source0, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, source0);

//        // Step Find_Contours0:
//        findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(source0, findContoursExternalOnly, findContoursOutput);
        HashMap<Point, Double> filteredContours = new HashMap<>();
        double highestCircularity = 0;
        for(MatOfPoint contour : findContoursOutput){
            double area = Imgproc.contourArea(contour);
            double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double circularity = 4*Math.PI * area / Math.pow(perimeter, 2);
            if(circularity >= 0.75 && circularity > highestCircularity){
                Moments M = Imgproc.moments(contour);
                int cX = (int)(M.m10 / M.m00);
                int cY = (int)(M.m01 / M.m00);
                Point point = new Point(cX, cY);
                filteredContours.put(point, circularity);
                highestCircularity = circularity;
            }
        }
        for(Map.Entry<Point, Double> entry : filteredContours.entrySet()){
            if(entry.getValue() >= highestCircularity){
                detectedCircle = entry.getKey();
                Imgproc.circle(source0Copy, entry.getKey(), 5, new Scalar(255, 0,0), -1);
                break;
            }
        }
        if(highestCircularity == 0){
            detectedCircle = null;
        }
//        blurInput.release();
//        blurOutput.release();
//        hsvThresholdInput.release();
//        hsvThresholdOutput.release();
//        source0.release();
        return source0Copy;
    }

    /**
     * This method is a generated getter for the output of a Blur.
     * @return Mat output from Blur.
     */

    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
//    public Mat hsvThresholdOutput() {
//        return hsvThresholdOutput;
//    }

    /**
     * This method is a generated getter for the output of a Find_Contours.
     * @return ArrayList<MatOfPoint> output from Find_Contours.
     */
//    public ArrayList<MatOfPoint> findContoursOutput() {
//        return findContoursOutput;
//    }


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
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
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
//        hierarchy.release();
    }




}