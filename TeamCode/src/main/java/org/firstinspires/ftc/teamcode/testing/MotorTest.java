//package org.firstinspires.ftc.teamcode.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.checkerframework.checker.units.qual.A;
//
//@TeleOp
//public class MotorTest extends LinearOpMode {
//       // private D
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        String[] names = new String[]{
//                "frontRight",
//                "backRight",
//                "frontLeft",
//                "backLeft"
//        };
//
//
//        DcMotor[] motors = new DcMotor[]{
//                hardwareMap.dcMotor.get("frontRight"),
//                hardwareMap.dcMotor.get("backRight"),
//                hardwareMap.dcMotor.get("frontLeft"),
//                hardwareMap.dcMotor.get("backLeft")
//        };
//        DcMotor saveMotor = hardwareMap.dcMotor.get("saveMotor");
//
//
//
//        waitForStart();
//        while (opModeIsActive()){
//
//
//
//            telemetry.update();
//            motor.setPower(gamepad2.left_stick_y);
//            telemetry.addData("CurrentPos", motor.getCurrentPosition());
//            telemetry.update();
////            for(DcMotor motor : motors){
////                motor.setPower(1);
////                Thread.sleep(5000);
////                motor.setPower(0);
////            }
//
//        }
//    }
//}
