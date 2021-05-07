package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.tnwutil.collections.Triplet;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Vector;

@Config
@Autonomous(group = "drive")
public class RoadrunnerCopyTest extends LinearOpMode {
    Vector<Triplet<Long, Pose2d, Vector<Triplet<Integer, Integer, Double>>>> recordedData;  // [(time delta, robot pose, [(type, index, value)])]

    // Joystick buttons and axes
    enum JoystickMapping {
        // Gamepad 1 axes
        GAMEPAD_1_LEFT_STICK_X,
        GAMEPAD_1_LEFT_STICK_Y,
        GAMEPAD_1_RIGHT_STICK_X,
        GAMEPAD_1_RIGHT_STICK_Y,
        GAMEPAD_1_LEFT_TRIGGER,
        GAMEPAD_1_RIGHT_TRIGGER,

        // Gamepad 1 buttons
        GAMEPAD_1_A,
        GAMEPAD_1_B,
        GAMEPAD_1_X,
        GAMEPAD_1_Y,
        GAMEPAD_1_LEFT_BUMPER,
        GAMEPAD_1_RIGHT_BUMPER,
        GAMEPAD_1_LEFT_STICK_BUTTON,
        GAMEPAD_1_RIGHT_STICK_BUTTON,

        // Gamepad 2 axes
        GAMEPAD_2_LEFT_STICK_X,
        GAMEPAD_2_LEFT_STICK_Y,
        GAMEPAD_2_RIGHT_STICK_X,
        GAMEPAD_2_RIGHT_STICK_Y,
        GAMEPAD_2_LEFT_TRIGGER,
        GAMEPAD_2_RIGHT_TRIGGER,

        // Gamepad 1 buttons
        GAMEPAD_2_A,
        GAMEPAD_2_B,
        GAMEPAD_2_X,
        GAMEPAD_2_Y,
        GAMEPAD_2_LEFT_BUMPER,
        GAMEPAD_2_RIGHT_BUMPER,
        GAMEPAD_2_LEFT_STICK_BUTTON,
        GAMEPAD_2_RIGHT_STICK_BUTTON
    }

    // Joystick controls are applied by the following logic:
    // If the target object is of type index 0 (DcMotor without encoders), the axis value directly sets the power
    // If the target object is of type index 1 (DcMotor with encoders), the axis value increments or decrements a target position
    // If the target object is of type index 2 (Servo), the axis value increments or decrements a target position
    // If the target object is of type index 3 (DcMotorEx with encoders), the axis value sets a target velocity

    // If a target position is being incremented on DcMotor or DcMotorEx

    // Buttons are treated as axes that range only from 0 to 1 discreetly rather than from -1 to 0 to 1 continuously

    // Joystick bindings
    JoystickMapping translationXBinding = JoystickMapping.GAMEPAD_1_LEFT_STICK_Y;  // Robot translational movement on the X axis
    JoystickMapping translationYBinding = JoystickMapping.GAMEPAD_1_LEFT_STICK_X;  // Robot translational movement on the Y axis
    JoystickMapping headingBinding = JoystickMapping.GAMEPAD_1_RIGHT_STICK_X;  // Robot heading adjustment
    JoystickMapping clawBinding = JoystickMapping.GAMEPAD_2_LEFT_STICK_X;  // Robot claw servo target incrementing/decrementing
    JoystickMapping armBinding = JoystickMapping.GAMEPAD_2_RIGHT_STICK_X;  // Robot arm motor target incrementing/decrementing
    JoystickMapping intakeBinding = JoystickMapping.GAMEPAD_2_LEFT_BUMPER;  // Robot intake absolute power
    JoystickMapping bucketBinding = JoystickMapping.GAMEPAD_2_LEFT_STICK_Y;  // Robot bucket motor target incrementing/decrementing
    JoystickMapping fingerBinding = JoystickMapping.GAMEPAD_2_RIGHT_STICK_Y;  // Robot finger servo target incrementing/decrementing
    JoystickMapping shooterBinding = JoystickMapping.GAMEPAD_1_RIGHT_STICK_Y;  // Robot shooter motor target velocity incrementing/decrementing
    JoystickMapping timeAdvanceBinding = JoystickMapping.GAMEPAD_1_RIGHT_TRIGGER;  // Trigger that decides how much to advance time by
    JoystickMapping timeReverseBinding = JoystickMapping.GAMEPAD_1_LEFT_TRIGGER;  // Trigger that decides how much to reverse time by (inverse of the previous, can only reverse time to the previous keyframe [FIXME] or time 0)

    double horizontalMovement;  // Applied by Roadrunner's functions
    double verticalMovement;  // Applied by Roadrunner's functions
    double headingMovement;  // Applied by Roadrunner's functions
    double clawTargetPosition;
    double armTargetPosition;  // Relative to the arm starting position, measured in integers but stored this way for consistency
    double intakePower;
    double bucketTargetPosition;  // Relative to the bucket starting position, measured in integers but stored this way for consistency
    double fingerTargetPosition;
    double shooterTargetVelocity;

    double horizontalPositionDelta;
    double verticalPositionDelta;
    double headingPositionDelta;

    long startTime;
    long deltaTime;
    long lastTime;
    long currentTimeDelta;  // Measured from startTime, which is set on RunOpMode

    Pose2d lastPose;
    Pose2d thisPose;

    Servo claw;
    Servo finger;
    DcMotor intake;
    DcMotor bucket;
    DcMotorEx shooter;
    DcMotor arm;

    boolean running = true;

    double joystickIndexToAxisValue(JoystickMapping joystickMapping) {  // Accepts an integer representing a joystick input index and returns the value of the joystick axis that corresponds to that index
        // If the index given corresponds to a button value, the return value will be 0 if the button is released and 1 if it is pressed
        // Returns NaN if no mapping was found

        switch (joystickMapping) {
            // Gamepad 1 axes
            case GAMEPAD_1_LEFT_STICK_X:
                return -gamepad1.left_stick_x;
            case GAMEPAD_1_LEFT_STICK_Y:
                return -gamepad1.left_stick_y;
            case GAMEPAD_1_RIGHT_STICK_X:
                return -gamepad1.right_stick_x;
            case GAMEPAD_1_RIGHT_STICK_Y:
                return gamepad1.right_stick_y;
            case GAMEPAD_1_LEFT_TRIGGER:
                return gamepad1.left_trigger;
            case GAMEPAD_1_RIGHT_TRIGGER:
                return gamepad1.right_trigger;

            // Gamepad 1 buttons
            case GAMEPAD_1_A:
                return gamepad1.a ? 1.0 : 0.0;
            case GAMEPAD_1_B:
                return gamepad1.b ? 1.0 : 0.0;
            case GAMEPAD_1_X:
                return gamepad1.x ? 1.0 : 0.0;
            case GAMEPAD_1_Y:
                return gamepad1.y ? 1.0 : 0.0;
            case GAMEPAD_1_LEFT_BUMPER:
                return gamepad1.left_bumper ? 1.0 : 0.0;
            case GAMEPAD_1_RIGHT_BUMPER:
                return gamepad1.right_bumper ? 1.0 : 0.0;
            case GAMEPAD_1_LEFT_STICK_BUTTON:
                return gamepad1.left_stick_button ? 1.0 : 0.0;
            case GAMEPAD_1_RIGHT_STICK_BUTTON:
                return gamepad1.right_stick_button ? 1.0 : 0.0;

            // Gamepad 2 axes
            case GAMEPAD_2_LEFT_STICK_X:
                return gamepad2.left_stick_x;
            case GAMEPAD_2_LEFT_STICK_Y:
                return gamepad2.left_stick_y;
            case GAMEPAD_2_RIGHT_STICK_X:
                return gamepad2.right_stick_x;
            case GAMEPAD_2_RIGHT_STICK_Y:
                return gamepad2.right_stick_y;
            case GAMEPAD_2_LEFT_TRIGGER:
                return gamepad2.left_trigger;
            case GAMEPAD_2_RIGHT_TRIGGER:
                return gamepad2.right_trigger;

            // Gamepad 2 buttons
            case GAMEPAD_2_A:
                return gamepad2.a ? 1.0 : 0.0;
            case GAMEPAD_2_B:
                return gamepad2.b ? 1.0 : 0.0;
            case GAMEPAD_2_X:
                return gamepad2.x ? 1.0 : 0.0;
            case GAMEPAD_2_Y:
                return gamepad2.y ? 1.0 : 0.0;
            case GAMEPAD_2_LEFT_BUMPER:
                return gamepad2.left_bumper ? 1.0 : 0.0;
            case GAMEPAD_2_RIGHT_BUMPER:
                return gamepad2.right_bumper ? 1.0 : 0.0;
            case GAMEPAD_2_LEFT_STICK_BUTTON:
                return gamepad2.left_stick_button ? 1.0 : 0.0;
            case GAMEPAD_2_RIGHT_STICK_BUTTON:
                return gamepad2.right_stick_button ? 1.0 : 0.0;
        }

        return Double.NaN;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        startTime = System.currentTimeMillis();
        lastTime = startTime;

        telemetry.addLine("Initializing non-movement-related motors and servos");

        claw = hardwareMap.get(Servo.class, "WGClaw");
        finger = hardwareMap.get(Servo.class, "ringFinger");
        intake = hardwareMap.get(DcMotor.class, "intakeDrive");
        bucket = hardwareMap.get(DcMotor.class, "ringElevator");
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        arm = hardwareMap.get(DcMotor.class, "WGArm");

        claw.setPosition(0.48);
        clawTargetPosition = 0.48;

        finger.setPosition(0.75);
        fingerTargetPosition = 0.75;

        bucket.setTargetPosition(bucket.getCurrentPosition());
        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucket.setPower(1.0);

        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initializing SampleMecanumDrive");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        thisPose = drive.getPoseEstimate();
        lastPose = thisPose;

        while (running) {
            // TODO: For non-servo-like values that are incremented (specifically the shooter target velocity), there should be a way to zero the value

            deltaTime = System.currentTimeMillis() - lastTime;

            horizontalMovement = joystickIndexToAxisValue(translationXBinding);
            verticalMovement = joystickIndexToAxisValue(translationYBinding);
            headingMovement = joystickIndexToAxisValue(headingBinding);
            clawTargetPosition += joystickIndexToAxisValue(clawBinding) * deltaTime / 1000;
            armTargetPosition += joystickIndexToAxisValue(armBinding) * deltaTime / 1000;
            intakePower = joystickIndexToAxisValue(intakeBinding);
            bucketTargetPosition += joystickIndexToAxisValue(bucketBinding) * deltaTime / 1000;
            fingerTargetPosition += joystickIndexToAxisValue(fingerBinding) * deltaTime / 1000;
            shooterTargetVelocity += joystickIndexToAxisValue(shooterBinding);
            currentTimeDelta += joystickIndexToAxisValue(timeAdvanceBinding) * deltaTime / 1000;
            currentTimeDelta -= joystickIndexToAxisValue(timeReverseBinding) * deltaTime / 1000;

            claw.setPosition(clawTargetPosition);
            finger.setPosition(fingerTargetPosition);
            intake.setPower(intakePower);
            bucket.setTargetPosition((int) bucketTargetPosition);
            shooter.setVelocity(shooterTargetVelocity);
            arm.setTargetPosition((int) armTargetPosition);

            drive.setDrivePower(new Pose2d(horizontalMovement, verticalMovement, headingMovement));

            thisPose = drive.getPoseEstimate();

            horizontalPositionDelta += (thisPose.getX() - lastPose.getX());
            verticalPositionDelta += (thisPose.getY() - lastPose.getY());
            headingPositionDelta += (thisPose.getHeading() - lastPose.getHeading());

            if (gamepad1.left_stick_button) {  // FIXME: This probably shouldn't be a hard-coded binding
                // Set a keyframe
                //recordedData.add(new Triplet<>());
            }
        }
    }
}
