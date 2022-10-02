package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class IntakeProtype2 {



    private DcMotor motor;
    private GamepadEx operator;
    private LinearOpMode opMode;
    private Telemetry telemetry;

    public void intake(GamepadEx operator, HardwareMap  hardwareMap, LinearOpMode opMode){
        this.operator = operator;
        this.motor = hardwareMap.dcMotor.get("liftMotor");
        this.opMode = opMode;
    }

    public void intakePower(GamepadEx operator, HardwareMap  hardwareMap, LinearOpMode opMode){
        motor.setPower(operator.getLeftY());
        opMode.telemetry.addData("The power is:", motor.getPower());
    }
}
