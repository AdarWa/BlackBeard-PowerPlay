package org.firstinspires.ftc.teamcode.TeleOpDrive.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Sensor: BNO055 IMU", group = "Sensor")
public class IMUTest extends LinearOpMode
{
    @Override
    public void runOpMode() {
        BNO055IMU imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        // Loop and update the app
        Orientation angles;
        while (opModeIsActive()) {
            angles =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", getHeading(angles));
            telemetry.addData("fall", getFall(angles));
            telemetry.update();
        }
    }

    private double getHeading(Orientation angles){
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }
    private double getFall(Orientation angles){
        return AngleUnit.DEGREES.normalize(angles.secondAngle);
    }
}


