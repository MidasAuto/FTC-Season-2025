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
@Autonomous(name = "Auto_Spins", group = "Autonomous")
public class Auto_Dos_2 extends LinearOpMode {

    // Variables
    private MoveArm moveArm;
    private MoveServo moveServo;
    ////////hi/////////

    public class MoveServo {
        private Servo servo;

        public MoveServo(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "clawServo1"); // TODO: RENAME "servo1" TO YOUR CONFIG NAME FOR YOUR SERVO
        }

        public void setServoPosition(double position) {
            servo.setPosition(Range.clip(position, 0.0, 1.0)); // Clipping to valid servo range (0.0 to 1.0)
        }
    }

    public class MoveServoAction implements Action {
        private MoveServo moveServo;
        private double targetPosition;

        public MoveServoAction(MoveServo moveServo, double targetPosition) {
            this.moveServo = moveServo;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            moveServo.setServoPosition(targetPosition);
            return true; // Action is complete immediately after setting the position
        }
    }

    //-------------------------------------//

    public class MoveArm {
        private DcMotorEx motor1;
        private DcMotorEx motor2;
        private double targetPosition = 0;
        private static final double THRESHOLD = 10; // Define the threshold for reaching the target

        public MoveArm(HardwareMap hardwareMap) {
            motor1 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor"); // TODO: RENAME "motor1" TO YOUR CONFIG NAME FOR YOUR MOTOR
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2 = hardwareMap.get(DcMotorEx.class, "armVerticalMotor2");
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void rotateArm(double targetPosition) {
            this.targetPosition = targetPosition; // No clipping, no max ticks
            double currentPosition = motor1.getCurrentPosition();
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

    public class MoveArmAction implements Action {
        private MoveArm moveArm;
        private double targetPosition;

        public MoveArmAction(MoveArm moveArm, double targetPosition) {
            this.moveArm = moveArm;
            this.targetPosition = targetPosition; // No clipping
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            moveArm.rotateArm(targetPosition);
            return !moveArm.isAtTarget(); // Return false when the motor is at the target position
        }
    }

    @Override
    public void runOpMode() {

        // Setup the start pose for the robot
        Pose2d startPose = new Pose2d(1, 1, Math.toRadians(90));

        // Initialize the MecanumDrive and MoveArm objects
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        moveArm = new MoveArm(hardwareMap);
        moveServo = new MoveServo(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {

            // Create the MoveArmAction for moving the arm to a target position
            MoveArmAction ArmPosition1 = new MoveArmAction(moveArm, 2000);
            MoveArmAction ArmPosition2 = new MoveArmAction(moveArm, 3000);
            MoveServoAction ServoPosition1x1 = new MoveServoAction(moveServo, 0.5);
            // Execute the action to move the arm
            Actions.runBlocking(drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(34, 34), Math.toRadians(90))
                    .afterDisp(3, ArmPosition2)
                    .build());
        }
    }
}
