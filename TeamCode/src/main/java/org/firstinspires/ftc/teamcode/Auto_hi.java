package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "First Auto", group = "Autonomous")
public class Auto_hi extends LinearOpMode {

    // Variables
    private double Var1 = 0;
    public static double kP = 0.1;
    public static double kF = 0.4;
    Servo clawServo1 = null;
    Servo clawServo2 = null;
    Servo clawServo3 = null;
    private MoveArm moveArm;
    private MoveClaw3 moveClaw3;
    private MoveClaw moveClaw;
    private MoveMotor moveMotor;
    private MoveClaw2 moveClaw2;

    //---------------------------------------//

    public class MoveClaw2 {
        private Servo servo2;

        public MoveClaw2(HardwareMap hardwareMap) {
            servo2 = hardwareMap.get(Servo.class, "clawServo2");

        }

        public void setServoPosition(double position) {
            servo2.setPosition(Range.clip(position, 0.0, 1.0)); // Clipping to valid servo range (0.0 to 1.0)

        }
    }

    public class MoveClaw2Action implements Action {
        private MoveClaw2 moveClaw2;
        private double targetPosition;

        public MoveClaw2Action(MoveClaw2 moveClaw2, double targetPosition) {
            this.moveClaw2 = moveClaw2;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            moveClaw2.setServoPosition(targetPosition);
            return true; // Action is complete immediately after setting the position
        }
    }

    public class MoveMotor {
        private DcMotorEx motor1, motor2;
        private double targetPosition = 0;
        private static final double THRESHOLD = 10; // Define the threshold for reaching the target

        public MoveMotor(HardwareMap hardwareMap) {
            motor1 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor");
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void rotateMotor(double targetPosition) {
            this.targetPosition = targetPosition; // No clipping, no max ticks
            double currentPosition = motor1.getCurrentPosition() + motor2.getCurrentPosition();
            double error = targetPosition - currentPosition;

            // Set motor power based on the error direction
            double power = (error > 0) ? 0.5 : -0.5; // Set a fixed power value for simplicity (adjust as needed)

            motor1.setPower(power);
            motor2.setPower(power);
        }

        public boolean isAtTarget() {
            return Math.abs(targetPosition - motor1.getCurrentPosition()) <= THRESHOLD;
        }
    }

    public class MoveMotorAction implements Action {
        private MoveMotor moveMotor;
        private double targetPosition;

        public MoveMotorAction(MoveMotor moveMotor, double targetPosition) {
            this.moveMotor = moveMotor;
            this.targetPosition = targetPosition; // No clipping
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            moveMotor.rotateMotor(targetPosition);
            return !moveMotor.isAtTarget(); // Return false when the motor is at the target position
        }
    }

    //-------------------------------------//

    public class MoveClaw {
        private Servo servo;

        public MoveClaw(HardwareMap hardwareMap) {

            servo = hardwareMap.get(Servo.class, "clawServo1");
        }

        public void setServoPosition(double position) {
            servo.setPosition(Range.clip(position, 0.0, 1.0)); // Clipping to valid servo range (0.0 to 1.0)

        }
    }

    public class MoveClawAction implements Action {
        private MoveClaw moveClaw;
        private double targetPosition;

        public MoveClawAction(MoveClaw moveClaw, double targetPosition) {
            this.moveClaw = moveClaw;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            moveClaw.setServoPosition(targetPosition);
            return true; // Action is complete immediately after setting the position
        }
    }
    private static final int MAX_TICKS = 2000;
    public static final int THRESHOLD = 80;
    // MoveArm class for controlling the arm motors
    public class MoveClaw3 {
        private Servo servo3;

        public MoveClaw3(HardwareMap hardwareMap) {
            servo3 = hardwareMap.get(Servo.class, "clawServo3");
        }

        public void setServoPosition(double position) {
            servo3.setPosition(Range.clip(position, 0.0, 1.0)); // Clipping to valid servo range (0.0 to 1.0)
        }
    }

    public class MoveClaw3Action implements Action {
        private MoveClaw3 moveClaw3;
        private double targetPosition;

        public MoveClaw3Action(MoveClaw3 moveClaw3, double targetPosition) {
            this.moveClaw3 = moveClaw3;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            moveClaw3.setServoPosition(targetPosition);
            return true; // Action is complete immediately after setting the position
        }
    }

    ////////hi/////////

    public class Killjoy {
        private DcMotorEx armMotor, armMotor2;

        public Killjoy(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armVerticalMotor");
            armMotor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

            armMotor.setPower(1);
            armMotor2.setPower(1);
        }
    }

    public class xKilljoy {
        private DcMotorEx armMotor, armMotor2;

        public xKilljoy(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armVerticalMotor");
            armMotor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

            armMotor.setPower(0);
            armMotor2.setPower(0);
        }
    }
    public class MoveArm {
        private DcMotorEx armMotor, armMotor2;

        private double targetPosition = 0;
        public MoveArm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armVerticalMotor");
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            armMotor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

            clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
            clawServo2 = hardwareMap.get(Servo.class, "clawServo2");
            clawServo3 = hardwareMap.get(Servo.class, "clawServo3");
        }

        public void rotateArm(double targetPosition) {
            targetPosition = Range.clip(targetPosition, 0, MAX_TICKS);
            double currentPosition = armMotor.getCurrentPosition() + armMotor2.getCurrentPosition();
            double error = targetPosition - currentPosition;
            double power = 1;

            if (error < 10) {
                power *= -1;
            } else if (error > 10) {
                power *= 1;
            }

            power = Range.clip(power, -1.0, 1.0);
            armMotor.setPower(power);
            armMotor2.setPower(power);

            if (isAtTarget()) {
                armMotor.setPower(0);
                armMotor2.setPower(0);
            }
        }

        public boolean isAtTarget() {
            return Math.abs(targetPosition - armMotor.getCurrentPosition()) <= THRESHOLD;
        }
        public class MoveMotor {
            private DcMotorEx motor1, motor2;
            private double targetPosition = 0;
            private static final double THRESHOLD = 10; // Define the threshold for reaching the target

            public MoveMotor(HardwareMap hardwareMap) {
                motor1 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor");
                motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");
                motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            public void rotateMotor(double targetPosition) {
                this.targetPosition = targetPosition; // No clipping, no max ticks
                double currentPosition = motor1.getCurrentPosition();
                double error = targetPosition - currentPosition;
                double power = (error > 0) ? 0.1: -0.1;

                motor1.setPower(power);
                motor2.setPower(power);
            }

            public boolean isAtTarget() {
                //return Math.abs(targetPosition - motor1.getCurrentPosition()) <= 80;
                return (targetPosition - motor1.getCurrentPosition()) * (targetPosition - motor1.getCurrentPosition()) <= 100;
            }
        }

        public class MoveMotorAction implements Action {
            private MoveMotor moveMotor;
            private double targetPosition;

            public MoveMotorAction(MoveMotor moveMotor, double targetPosition) {
                this.moveMotor = moveMotor;
                this.targetPosition = targetPosition; // No clipping
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveMotor.rotateMotor(targetPosition);
                return !moveMotor.isAtTarget(); // Return false when the motor is at the target position
            }
        }
    }

    // MoveArmAction class for integrating arm movement into RoadRunner
    public class MoveArmAction implements Action {
        private MoveArm moveArm;
        private double targetPosition;

        public MoveArmAction(MoveArm moveArm, double targetPosition) {
            this.moveArm = moveArm;
            this.targetPosition = Range.clip(targetPosition, 0, MAX_TICKS);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            moveArm.rotateArm(targetPosition);
            boolean isAtTarget = moveArm.isAtTarget();
            telemetry.addData("MoveArmAction", "At Target: %b, Target Position: %.2f", isAtTarget, targetPosition);
            telemetry.update();
            return !isAtTarget; // Return false when the arm is at the target position
        }
    }

    @Override
    public void runOpMode() {

        // Setup the start pose for the robot
        Pose2d startPose = new Pose2d(84, 20, Math.toRadians(90));

        // Initialize the MecanumDrive and MoveArm objects
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        moveArm = new MoveArm(hardwareMap);
        moveClaw3 = new MoveClaw3(hardwareMap);
        moveClaw = new MoveClaw(hardwareMap);
        moveClaw2 = new MoveClaw2(hardwareMap);
        moveMotor = new MoveMotor(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {

            // Create the MoveArmAction for moving the arm to a target position
            MoveClawAction ClawYAW1 = new MoveClawAction(moveClaw, 0.35);
            MoveClaw2Action ClawClosed = new MoveClaw2Action(moveClaw2, 0.9);
            MoveClaw3Action ClawClosed2 = new MoveClaw3Action(moveClaw3, 0.2);
            MoveClaw2Action ClawOpen = new MoveClaw2Action(moveClaw2, 0.6);
            MoveClaw3Action ClawOpen2 = new MoveClaw3Action(moveClaw3, 0.4);
            MoveMotorAction MotorPosition = new MoveMotorAction(moveMotor, 8000);
            // Execute the action to move the arm
            Actions.runBlocking(drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(84, 41.5), Math.toRadians(90),
                            new TranslationalVelConstraint(80),
                            new ProfileAccelConstraint(-80, 80))
                    .afterDisp(1, MotorPosition)
                    .afterDisp(1, ClawClosed)
                    .afterDisp(1, ClawClosed2)
                    .afterDisp(1, ClawYAW1)
                    .waitSeconds(3)
                    .afterTime(2, ClawOpen)
                    .afterTime(2, ClawOpen2)
                    .strafeToLinearHeading(new Vector2d(84, 19), Math.toRadians(90),
                            new TranslationalVelConstraint(80),
                            new ProfileAccelConstraint(-80, 80))
                    .strafeToLinearHeading(new Vector2d(40, 30), Math.toRadians(90),
                            new TranslationalVelConstraint(80),
                            new ProfileAccelConstraint(-80, 80))
                    .build());
        }
    }
}

