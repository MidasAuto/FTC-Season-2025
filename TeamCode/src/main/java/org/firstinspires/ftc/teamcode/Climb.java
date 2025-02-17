package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Climb")
public class Climb extends OpMode {
    DcMotor armVerticalMotor;
    DcMotor armVerticalMotor2;
    DcMotor armExtensionMotor;

    Servo clawServo1 = null;
    Servo clawServo2 = null;
    Servo clawServo3 = null;

    @Override
    public void init() {

        armVerticalMotor = hardwareMap.get(DcMotor.class, "armVerticalMotor");
        armVerticalMotor2 = hardwareMap.get(DcMotor.class, "armVerticalMotor2");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "armExpansionMotor");

        clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");
        clawServo3 = hardwareMap.get(Servo.class, "clawServo3");

        armVerticalMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        armExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawServo2.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad2.a) {
            armVerticalMotor.setPower(1);
            armVerticalMotor2.setPower(1);
        } else if (gamepad2.b) {
            armVerticalMotor.setPower(0);
            armVerticalMotor2.setPower(0);
        }

    }
}