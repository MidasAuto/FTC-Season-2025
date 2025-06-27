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
@Autonomous(name = "Left", group = "Autonomous")
public class LeftAuto extends LinearOpMode {




    private MoveArm moveArm;
    private MoveServo moveServo;

//private MoveActuator moveActuator;



// ---------------------- MoveServo ----------------------






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




// ---------------------- MoveArm ----------------------
/*
public class MoveActuator {
    private DcMotorEx motor3;

    private static final double THRESHOLD = 15;

    public MoveActuator(HardwareMap hardwareMap) {
        motor3 = hardwareMap.get(DcMotorEx.class, "armExpansionMotor");
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void expandArm(double targetPostion) {
        double current = motor3.getCurrentPosition();
        double error = targetPosition - current;
        double power = (error > 0) ? 0.5 : -0.5;
        motor3.setPower(power);

    }

    public boolean isAtTarget(double target) {
        return Math.abs(target - motor3.getCurrentPosition()) <= THRESHOLD;
    }

    public void stopMotors() {
        motor3.setPower(0);
    }




    public double getCurrentPosition() {
        return motor3.getCurrentPosition();
    }
}

    public class MoveActuatorAction implements Action {
        private MoveActuator moveActuator;
        private double targetPosition;




        public MoveActuatorAction(MoveActuator moveActuator, double targetPosition) {
            this.moveActuator = moveActuator;
            this.targetPosition = targetPosition;
        }




        @Override
        public boolean run(TelemetryPacket packet) {
            if (!moveActuator.isAtTarget(targetPosition)) {
                moveActuator.expandArm(targetPosition);
                packet.put("Actuator Pos", moveArm.getCurrentPosition());
                packet.put("Target1", targetPosition);
                return true; // Still running
            } else {
                moveArm.stopMotors();
                return false; // Done
            }
        }
    }

*/

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
        private MoveServoAction servoAction; // Added servoAction
        private boolean armDone = false;
        private boolean servoDone = false;




        public ArmAndServoAction(MoveArm moveArm, MoveServo moveServo, double armTarget, double servoTarget) {
            this.armAction = new MoveArmAction(moveArm, armTarget);
            this.servoAction = new MoveServoAction(moveServo, servoTarget); // Initialize servoAction
        }




        @Override
        public boolean run(TelemetryPacket packet) {
            if (!armDone) {
                armDone = !armAction.run(packet);
            }
            if (!servoDone) {
                servoDone = !servoAction.run(packet); // Run servoAction
            }




            packet.put("Arm Done", armDone);
            packet.put("Servo Done", servoDone);




            return !(armDone && servoDone);
        }
    }




// ---------------------- Run Auto ----------------------




    @Override
    public void runOpMode() {


        moveArm = new MoveArm(hardwareMap);
        moveServo = new MoveServo(hardwareMap);


        moveServo.setServoPosition(0.9);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);




        moveArm = new MoveArm(hardwareMap);
        moveServo = new MoveServo(hardwareMap);




        waitForStart();

        if (opModeIsActive()) {

            // Raise arm and open claw at top rung
            ArmAndServoAction armHook = new ArmAndServoAction(moveArm, moveServo,1990, 0.9);

            ArmAndServoAction armRaise = new ArmAndServoAction(moveArm, moveServo, 1590, 0.9);

            ArmAndServoAction armLetgo = new ArmAndServoAction(moveArm, moveServo, 1500, 0.6);


            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeToLinearHeading(new Vector2d(-1.5, -26), Math.toRadians(90))
                            .afterDisp(1, armRaise)
                            .waitSeconds(1.3)
                            .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(90))
                            .afterDisp(1, armHook)
                            .waitSeconds(1.15)
                            .afterDisp(3,armLetgo)
                            .strafeToLinearHeading(new Vector2d(0, -26), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(32.5, -26), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(32.5, -55), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(44.5, -55), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(44.5, -3.5), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(43, -55), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(55, -55), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(55, -3.5), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(53, -55), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(61, -55), Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(61, -3.5), Math.toRadians(90))
                            .build()

            );
        }
    }
}