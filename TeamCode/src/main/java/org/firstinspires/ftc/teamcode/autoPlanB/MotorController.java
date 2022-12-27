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
        motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

//    for (int i = 0; i < motors.length; i+=2) {
//
//    }

    public void forward(double cm, double power){
        for(DcMotor m : motors){
            m.setPower(power);
        }
        sleep(cm);
        for(DcMotor m : motors){
            m.setPower(0);
        }
    }

    private void sleep(double sec){
        try {
            Thread.sleep((long)(sec*1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }



    public void backward(double cm, double power){

    }

    public void strafeLeft(double cm, double power){
        motors[1].setPower(power);
        motors[3].setPower(-power);
        motors[0].setPower(-power);
        motors[2].setPower(power);
        sleep(cm);
        for(DcMotor m : motors){
            m.setPower(0);
        }
    }

    public void strafeRight(double cm, double power){
        motors[1].setPower(-power);
        motors[3].setPower(power);
        motors[0].setPower(power);
        motors[2].setPower(-power);
        sleep(cm);
        for(DcMotor m : motors){
            m.setPower(0);
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
