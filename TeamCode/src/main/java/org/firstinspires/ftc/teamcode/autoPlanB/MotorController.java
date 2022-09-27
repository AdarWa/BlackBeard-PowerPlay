package org.firstinspires.ftc.teamcode.autoPlanB;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorController {

    public DcMotor[] motors;

    public LinearOpMode opMode;

    public static final double ticksPerCm = 0;

    //frontLeft, frontRight, backLeft, backRight
    public MotorController(DcMotor[] motors, LinearOpMode opMode){
        this.motors = motors;
    }

//    for (int i = 0; i < motors.length; i+=2) {
//
//    }

    public void forward(double cm, double power){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setTargetPosition((int)(cm * ticksPerCm));

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor.setPower(power);
        }
        for (DcMotor motor : motors) {
            while (motor.isBusy()) {
                opMode.idle();
            }
        }
    }


    public void backward(double cm, double power){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setTargetPosition((int)(cm * ticksPerCm));

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor.setPower(-power);
        }
        for (DcMotor motor : motors) {
            while (motor.isBusy()) {
                opMode.idle();
            }
        }
    }

    public void diagonalUpLeft(double cm, double power){
        for (int i = 1; i < motors.length; i+=2) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motors[i].setTargetPosition((int)(cm * ticksPerCm));

            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motors[i].setPower(-power);
        }
        for (int i = 1; i < motors.length; i+=2) {
            while (motors[i].isBusy())
                opMode.idle();
        }
    }




}
