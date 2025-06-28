package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
@Autonomous(name = "Right", group = "Autonomous")
public class RightAuto extends LinearOpMode {

    private MoveArm moveArm;
    private MoveServo moveServo;

    private class MoveServo {
        private Servo servo;
        private Servo servo1;

        public MoveServo(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "clawServo1");
            servo1 = hardwareMap.get(Servo.class, "clawServo2");
            servo1.setDirection(Servo.Direction.REVERSE);
        }

        public void setServoPosition(double position) {
            servo.setPosition(Range.clip(position, 0.0, 1.0));
            servo1.setPosition(Range.clip(position, 0.0, 1.0));
        }
    }

    private class MoveServoAction implements Action {
        private MoveServo moveServo;
        private double targetPosition;
        private boolean done = false;

        public MoveServoAction(MoveServo moveServo, double targetPosition) {
            this.moveServo = moveServo;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!done) {
                moveServo.setServoPosition(targetPosition);
                done = true;
            }
            return false; // Done after first run
        }
    }

    public class MoveArm {
        private DcMotorEx motor1;
        private DcMotorEx motor2;
        private static final double THRESHOLD = 15;
        private static final double MAX_POWER = 4.5;
        private static final double kP = 0.002; // Tune as needed

        public MoveArm(HardwareMap hardwareMap) {
            motor1 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor");
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void rotateArm(double targetPosition) {
            double current = motor1.getCurrentPosition();
            double error = targetPosition - current;
            double power = Range.clip(error * kP, -MAX_POWER, MAX_POWER);
            motor1.setPower(power);
            motor2.setPower(power);
        }

        public boolean isAtTarget(double target) {
            return Math.abs(target - motor1.getCurrentPosition()) <= THRESHOLD;
        }

        public void stopMotors() {
            motor1.setPower(0);
            motor2.setPower(0);
        }

        public double getCurrentPosition() {
            return motor1.getCurrentPosition();
        }
    }

    public class MoveArmAction implements Action {
        private MoveArm moveArm;
        private double targetPosition;

        public MoveArmAction(MoveArm moveArm, double targetPosition) {
            this.moveArm = moveArm;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!moveArm.isAtTarget(targetPosition)) {
                moveArm.rotateArm(targetPosition);
                packet.put("Arm Pos", moveArm.getCurrentPosition());
                packet.put("Target", targetPosition);
                return true; // Still running
            } else {
                moveArm.stopMotors();
                return false; // Done
            }
        }
    }

    public class ArmAndServoAction implements Action {
        private MoveArmAction armAction;
        private MoveServoAction servoAction;
        private boolean armDone = false;
        private boolean servoDone = false;

        public ArmAndServoAction(MoveArm moveArm, MoveServo moveServo, double armTarget, double servoTarget) {
            this.armAction = new MoveArmAction(moveArm, armTarget);
            this.servoAction = new MoveServoAction(moveServo, servoTarget);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!armDone) {
                armDone = !armAction.run(packet);
            }
            if (!servoDone) {
                servoDone = !servoAction.run(packet);
            }
            packet.put("Arm Done", armDone);
            packet.put("Servo Done", servoDone);
            return !(armDone && servoDone);
        }
    }

    @Override
    public void runOpMode() {
        moveArm = new MoveArm(hardwareMap);
        moveServo = new MoveServo(hardwareMap);

        moveServo.setServoPosition(0.9);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        if (opModeIsActive()) {
            ArmAndServoAction armHook = new ArmAndServoAction(moveArm, moveServo, 1995, 0.9);
            ArmAndServoAction armRaise = new ArmAndServoAction(moveArm, moveServo, 1500, 0.9);
            ArmAndServoAction armLetgo = new ArmAndServoAction(moveArm, moveServo, 1990, 0.6);
            ArmAndServoAction armGrab = new ArmAndServoAction(moveArm, moveServo, 4700, 0.6);
            ArmAndServoAction armGrabClose = new ArmAndServoAction(moveArm, moveServo, 4700, 0.9);
            ArmAndServoAction armBack = new ArmAndServoAction(moveArm, moveServo, 1600, 0.9);
            ArmAndServoAction armClose = new ArmAndServoAction(moveArm, moveServo, 5250, 0.9);

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeToLinearHeading(new Vector2d(0, -20), Math.toRadians(90))
                            .afterDisp(1, armRaise)
                            .waitSeconds(0.5)
                            .strafeToLinearHeading(new Vector2d(2, -33), Math.toRadians(90))
                            .afterDisp(1, armHook)
                            .strafeToLinearHeading(new Vector2d(2.5, -33), Math.toRadians(90))
                            .waitSeconds(0.5)
                            .afterDisp(3, armLetgo)
                            .strafeToLinearHeading(new Vector2d(0, -26), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(-45, -26), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(-46, -55), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(-57, -55), Math.toRadians(90))
                            .waitSeconds(0.25)
                            .strafeToLinearHeading(new Vector2d(-57, -4), Math.toRadians(90))
                            .afterDisp(0, armGrab)
                            .strafeToLinearHeading(new Vector2d(-42, -25), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(-42, -14.5), Math.toRadians(90))
                            .afterDisp(0, armGrabClose)
                            .waitSeconds(1)
                            .afterDisp(0, armBack)
                            .strafeToLinearHeading(new Vector2d(0, -12), Math.toRadians(90))
                            .afterDisp(0, armRaise)
                            .waitSeconds(0.25)
                            .strafeToLinearHeading(new Vector2d(1, -12), Math.toRadians(90))
                            .afterDisp(1, armHook)
                            .waitSeconds(0.25)
                            .strafeToLinearHeading(new Vector2d(20, -25), Math.toRadians(90))
                            .afterDisp(3, armLetgo)
                            .strafeToLinearHeading(new Vector2d(20, -33), Math.toRadians(90))
                            .afterDisp(1,armClose)
                            .build());

        }
    }
}
