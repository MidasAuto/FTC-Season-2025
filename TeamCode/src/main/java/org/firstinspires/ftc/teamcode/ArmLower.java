package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ArmLower")
public class ArmLower extends OpMode {

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor armVerticalMotor;
    DcMotor armVerticalMotor2;
    DcMotor armExtensionMotor;

    Servo clawServo1 = null;
    Servo clawServo2 = null;
    Servo clawServo3 = null;

    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        armVerticalMotor = hardwareMap.get(DcMotor.class, "armVerticalMotor");
        armVerticalMotor2 = hardwareMap.get(DcMotor.class, "armVerticalMotor2");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "armExpansionMotor");

        clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");
        clawServo3 = hardwareMap.get(Servo.class, "clawServo3");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armVerticalMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        armExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double verticalPower = -gamepad2.right_stick_y;
        double motorSpeed = 1;

        if (gamepad1.left_stick_button) {
            motorSpeed /= 3;
        }

        double drive = gamepad1.left_stick_y * motorSpeed;
        double strafe = gamepad1.left_stick_x * motorSpeed;
        double turn = gamepad1.right_stick_x * 0.5 * motorSpeed;
        double fRightPower = drive + turn + strafe;
        double fLeftPower = drive - turn - strafe;
        double bRightPower = drive + turn - strafe;
        double bLeftPower = drive - turn + strafe;

        armVerticalMotor.setPower(-0.25);
        armVerticalMotor2.setPower(-0.25);




    }
}