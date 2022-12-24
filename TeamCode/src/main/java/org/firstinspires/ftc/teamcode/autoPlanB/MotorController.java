package org.firstinspires.ftc.teamcode.autoPlanB;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class MotorController {

    public DcMotor[] motors;

    public LinearOpMode opMode;

    public static final double ticksPerCm = 6.666666667;

    //frontLeft, frontRight, backLeft, backRight
    public MotorController(DcMotor[] motors, LinearOpMode opMode){
        this.opMode = opMode;
        this.motors = motors;
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        for(DcMotor m : motors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
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

    public void strafeLeft(double cm, double power){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setTargetPosition((int)(cm * ticksPerCm));

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        motors[1].setPower(power);
        motors[3].setPower(-power);
        motors[0].setPower(-power);
        motors[2].setPower(power);
        for (DcMotor motor : motors) {
            while (motor.isBusy()) {
                opMode.idle();
            }
        }
    }

    public void strafeRight(double cm, double power){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setTargetPosition((int)(cm * ticksPerCm));

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        motors[1].setPower(-power);
        motors[3].setPower(power);
        motors[0].setPower(power);
        motors[2].setPower(-power);
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
