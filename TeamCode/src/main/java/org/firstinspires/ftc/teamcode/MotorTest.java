package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestServoMotor", group="Linear OpMode")
public class MotorTest extends LinearOpMode {

    private DcMotor testMotor = null;
    private Servo testServo = null;

    @Override
    public void runOpMode() {
        // this is for testing the motor
//        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
//        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        waitForStart();
//
//        testMotor.setPower(0.5);
//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        testMotor.setPower(-0.5);
        testServo = hardwareMap.get(Servo.class, "testServo");
        testServo.setPosition(0);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry.addData("Servo Position", testServo.getPosition());
        testServo.setPosition(1);
        telemetry.addData("Servo Position", testServo.getPosition());
    }
}
