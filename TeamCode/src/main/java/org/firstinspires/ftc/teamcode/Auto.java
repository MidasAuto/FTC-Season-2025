package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "Gpt", group = "Autonomous")
public class Gpt extends LinearOpMode {

    private MoveArm moveArm;
    private MoveServo moveServo;

    // ---------------------- MoveServo ----------------------

    private class MoveServo {
        private Servo servo;

        public MoveServo(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "clawServo1");
        }

        public void setServoPosition(double position) {
            servo.setPosition(Range.clip(position, 0.0, 1.0));
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

    private class MoveServo1 {
        private Servo servo1;

        public MoveServo1(HardwareMap hardwareMap) {
            servo1 = hardwareMap.get(Servo.class, "clawServo2");
        }

        public void setServoPosition(double position) {
            servo1.setPosition(Range.clip(position, 0.0, 1.0));
        }
    }

    // ---------------------- MoveArm ----------------------

    public class MoveArm {
        private DcMotorEx motor1;
        private DcMotorEx motor2;
        private static final double THRESHOLD = 15;

        public MoveArm(HardwareMap hardwareMap) {
            motor1 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor");
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void rotateArm(double targetPosition) {
            double current = motor1.getCurrentPosition();
            double error = targetPosition - current;
            double power = (error > 0) ? 0.5 : -0.5;
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

    // ---------------------- Combo Action ----------------------

    public class ArmAndServoAction implements Action {
        private MoveArmAction armAction;
        private boolean armDone = false;
        private boolean servoDone = false;

        public ArmAndServoAction(MoveArm moveArm, MoveServo moveServo, double armTarget, double servoTarget) {
            this.armAction = new MoveArmAction(moveArm, armTarget);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!armDone) armDone = !armAction.run(packet);

            packet.put("Arm Done", armDone);
            packet.put("Servo Done", servoDone);

            return !(armDone && servoDone);
        }
    }

    // ---------------------- Run Auto ----------------------

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(1, 1, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        moveArm = new MoveArm(hardwareMap);
        moveServo = new MoveServo(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {

            // Raise arm and open claw at top rung
            ArmAndServoAction armAndServoAction = new ArmAndServoAction(moveArm, moveServo,3700, 0.9);
            // Retract arm and close claw
            ArmAndServoAction retractArmAction = new ArmAndServoAction(moveArm, moveServo, 0, 0.0);

            ArmAndServoAction pickUpBlockAction = new ArmAndServoAction(moveArm, moveServo, 6000, 0.5);

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            // Move to top rung
                            .strafeToLinearHeading(new Vector2d(0, 10), Math.toRadians(90))
                            .afterDisp(3, armAndServoAction).waitSeconds(1)
                            .strafeToLinearHeading(new Vector2d(0, 27), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(0, 23),Math.toRadians(90))


                            // Move behind other peices and push
                            .strafeToLinearHeading(new Vector2d(40, 20), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(40, 50), Math.toRadians(90))
                            .afterDisp(3, pickUpBlockAction)
                            .strafeToLinearHeading(new Vector2d(45, 40), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(45, 15), Math.toRadians(90))

                            .build()
            );
        }
    }
}
