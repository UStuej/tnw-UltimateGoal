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
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(group = "drive")
public class RegionalsRoadrunnerTest extends LinearOpMode {

    char autoCase = 'X';


    // ROADRUNNER VALUES:
    // Constant Roadrunner Pose Values
    // Starting pose
    final Pose2d STARTING_POSE = new Pose2d(-63, -32, Math.toRadians(0));

    // Target zone poses
    final int DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING = 10; // inches
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_A = new Pose2d(-35, -52, Math.toRadians(15));
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_B = new Pose2d(-35, -52, Math.toRadians(15));
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_C = new Pose2d(-35 - 4, -55 - 4, Math.toRadians(330));
    final Pose2d SECOND_WOBBLE_GOAL_ALIGN_POSITION = new Pose2d(SECOND_WOBBLE_GOAL_PICKUP_POSITION_B.getX() + 12, SECOND_WOBBLE_GOAL_PICKUP_POSITION_B.getY() + 12, SECOND_WOBBLE_GOAL_PICKUP_POSITION_B.getHeading());
    final Pose2d SECOND_WOBBLE_GOAL_ALIGN_POSITION_C = new Pose2d(SECOND_WOBBLE_GOAL_PICKUP_POSITION_C.getX() + 12, SECOND_WOBBLE_GOAL_PICKUP_POSITION_C.getY(), SECOND_WOBBLE_GOAL_PICKUP_POSITION_C.getHeading());
    final Pose2d TARGET_ZONE_A1 = new Pose2d(24, -51, Math.toRadians(90));
    final Pose2d TARGET_ZONE_A2 = new Pose2d(TARGET_ZONE_A1.getX(), TARGET_ZONE_A1.getY() + DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_A1.getHeading());
    final Pose2d TARGET_ZONE_B1 = new Pose2d(30, -34, Math.toRadians(180));
    final Pose2d TARGET_ZONE_B2 = new Pose2d(TARGET_ZONE_B1.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_B1.getY() + 2, TARGET_ZONE_B1.getHeading());
    final Pose2d TARGET_ZONE_C1 = new Pose2d(53, -50, Math.toRadians(180));
    final Pose2d TARGET_ZONE_C2 = new Pose2d(TARGET_ZONE_C1.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_C1.getY(), TARGET_ZONE_C1.getHeading());
    // Starter stack related poses
    final Vector2d STARTER_STACK = new Vector2d(-24, -35);
    final Pose2d LONG_SHOT_POSE = new Pose2d(-37, -36, Math.toRadians(356));
    // Power shot related poses
    final Pose2d POWER_SHOT_SHOOT_1 = new Pose2d(-3, -3.5, Math.toRadians(356));
    final double DISTANCE_BETWEEN_POWER_SHOTS = 8; // inches
    // Parking pose
    final Pose2d PARKING_POSE = new Pose2d(12, -24, Math.toRadians(0));

    // MOTOR AND SERVO DECLARATION:
    // Intake motor
    private DcMotor intakeDrive;

    // Wobble Goal manipulation motors and servos
    private Servo wobbleClaw;  // Wobble goal claw servo
    private Servo fingerServo;  // Finger servo
    private DcMotor wobbleArm;  // Wobble goal arm motor (used with encoders and runToPosition, so it acts like a servo)

    // Ring shooter motor
    private DcMotorEx ringShooter;

    // Ring Elevator motor
    private DcMotor ringElevator;

    // MOTOR AND SERVO POSITION CONSTANTS:
    private static double highGoalTPS = 57.5 * 28;  // Ticks per second of the shooter when active and aiming for the high goal
    private static double powerShotTPS = 50 * 28; // Ticks per second of the shooter when active and aiming for the power shots

    private static double CLAW_OPENED_POSITION = 0.24;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.80;  // The position of the claw when it is closed

    private static int ARM_DOWN_POSITION_DELTA = 415;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's down
    private static int ARM_UP_POSITION_DELTA = 221;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up
    private static int ARM_HOVER_POSITION_DELTA = 330;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up

    private static int ARM_DOWN_POSITION;  // The absolute position (in motor encoder units) of the arm's down position. Set on init
    private static int ARM_UP_POSITION;  // The absolute position (in motor encoder units) of the arm's up position. Set on init
    private static int ARM_HOVER_POSITION;  // The absolute position (in motor encoder units) of the arm's hover position. Set on init
    private static int ARM_STARTING_POSITION; // The absolute position (in motor encoder units) of the arm's starting position. Set on init

    private static int RING_ELEVATOR_UP_POSITION;  // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION;  // The position of the Ring Elevator when it is in the DOWN state
    private static double RING_ELEVATOR_POWER = 0.7;  // The power for the motor to use when running to its target position

    private static double RING_FINGER_IN_POSITION = 0.23;  // The position of the finger servo when it's in
    private static double RING_FINGER_OUT_POSITION = 0.75;  // The position of the finger servo when it's out

    private static double INTAKE_IN_POWER = -1;  // The power set to intake drive for collecting rings
    private static double INTAKE_OUT_POWER = INTAKE_IN_POWER * -1;  // The power set to intake drive for ejecting rings


    // Gamepad 1 inputs JUST pressed
    private boolean gamepad1APressed = false;  // Whether or not the gamepad 1 a button was JUST pressed, handled by the handleInput function
    private boolean gamepad1BPressed = false;  // Whether or not the gamepad 1 b button was JUST pressed, handled by the handleInput function
    private boolean gamepad1XPressed = false;  // Whether or not the gamepad 1 x button was JUST pressed, handled by the handleInput function
    private boolean gamepad1YPressed = false;  // Whether or not the gamepad 1 y button was JUST pressed, handled by the handleInput function
    private boolean gamepad1LeftShoulderPressed = false;  // Whether or not the gamepad 1 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad1RightShoulderPressed = false;  // Whether or not the gamepad 1 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad1LeftStickPressed = false;  // Whether or not the gamepad 1 left stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad1RightStickPressed = false;  // Whether or not the gamepad 1 right stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadUpPressed = false;  // Whether or not the gamepad 1 dpad up button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadDownPressed = false;  // Whether or not the gamepad 1 dpad down button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadLeftPressed = false;  // Whether or not the gamepad 1 dpad left button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadRightPressed = false;  // Whether or not the gamepad 1 dpad right button was JUST pressed, handled by the handleInput function

    // Gamepad 1 inputs being held
    private boolean gamepad1AHeld = false;  // Whether or not the gamepad 1 a button is being held, handled by the handleInput function
    private boolean gamepad1BHeld = false;  // Whether or not the gamepad 1 b button is being held, handled by the handleInput function
    private boolean gamepad1XHeld = false;  // Whether or not the gamepad 1 x button is being held, handled by the handleInput function
    private boolean gamepad1YHeld = false;  // Whether or not the gamepad 1 y button is being held, handled by the handleInput function
    private boolean gamepad1LeftShoulderHeld = false;  // Whether or not the gamepad 1 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad1RightShoulderHeld = false;  // Whether or not the gamepad 1 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad1LeftStickHeld = false;  // Whether or not the gamepad 1 left stick button is being held, handled by the handleInput function
    private boolean gamepad1RightStickHeld = false;  // Whether or not the gamepad 1 right stick button is being held, handled by the handleInput function
    private boolean gamepad1DpadUpHeld = false;  // Whether or not the gamepad 1 dpad up button is being held, handled by the handleInput function
    private boolean gamepad1DpadDownHeld = false;  // Whether or not the gamepad 1 dpad down button is being held, handled by the handleInput function
    private boolean gamepad1DpadLeftHeld = false;  // Whether or not the gamepad 1 dpad left button is being held, handled by the handleInput function
    private boolean gamepad1DpadRightHeld = false;  // Whether or not the gamepad 1 dpad right button is being held, handled by the handleInput function

    // Gamepad 2 inputs JUST pressed
    private boolean gamepad2APressed = false;  // Whether or not the gamepad 2 a button was JUST pressed, handled by the handleInput function
    private boolean gamepad2BPressed = false;  // Whether or not the gamepad 2 b button was JUST pressed, handled by the handleInput function
    private boolean gamepad2XPressed = false;  // Whether or not the gamepad 2 x button was JUST pressed, handled by the handleInput function
    private boolean gamepad2YPressed = false;  // Whether or not the gamepad 2 y button was JUST pressed, handled by the handleInput function
    private boolean gamepad2LeftShoulderPressed = false;  // Whether or not the gamepad 2 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad2RightShoulderPressed = false;  // Whether or not the gamepad 2 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad2LeftStickPressed = false;  // Whether or not the gamepad 2 left stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad2RightStickPressed = false;  // Whether or not the gamepad 2 right stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad2DpadUpPressed = false;  // Whether or not the gamepad 1 dpad up button is being held, handled by the handleInput function
    private boolean gamepad2DpadDownPressed = false;  // Whether or not the gamepad 2 dpad down button was JUST pressed, handled by the handleInput function
    private boolean gamepad2DpadLeftPressed = false;  // Whether or not the gamepad 2 dpad left button was JUST pressed, handled by the handleInput function
    private boolean gamepad2DpadRightPressed = false;  // Whether or not the gamepad 2 dpad right button was JUST pressed, handled by the handleInput function

    // Gamepad 2 inputs being held
    private boolean gamepad2AHeld = false;  // Whether or not the gamepad 2 a button is being held, handled by the handleInput function
    private boolean gamepad2BHeld = false;  // Whether or not the gamepad 2 b button is being held, handled by the handleInput function
    private boolean gamepad2XHeld = false;  // Whether or not the gamepad 2 x button is being held, handled by the handleInput function
    private boolean gamepad2YHeld = false;  // Whether or not the gamepad 2 y button is being held, handled by the handleInput function
    private boolean gamepad2LeftShoulderHeld = false;  // Whether or not the gamepad 2 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad2RightShoulderHeld = false;  // Whether or not the gamepad 2 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad2LeftStickHeld = false;  // Whether or not the gamepad 2 left stick button is being held, handled by the handleInput function
    private boolean gamepad2RightStickHeld = false;  // Whether or not the gamepad 2 right stick button is being held, handled by the handleInput function
    private boolean gamepad2DpadUpHeld = false;  // Whether or not the gamepad 1 dpad up button is being held, handled by the handleInput function
    private boolean gamepad2DpadDownHeld = false;  // Whether or not the gamepad 2 dpad down button is being held, handled by the handleInput function
    private boolean gamepad2DpadLeftHeld = false;  // Whether or not the gamepad 2 dpad left button is being held, handled by the handleInput function
    private boolean gamepad2DpadRightHeld = false;  // Whether or not the gamepad 2 dpad right button is being held, handled by the handleInput function

    // Gamepad 1 axes
    private double gamepad1LeftStickX = 0.0;
    private double gamepad1LeftStickY = 0.0;
    private double gamepad1RightStickX = 0.0;
    private double gamepad1RightStickY = 0.0;
    private double gamepad1LeftTrigger = 0.0;
    private double gamepad1RightTrigger = 0.0;

    // Gamepad 2 axes
    private double gamepad2LeftStickX = 0.0;
    private double gamepad2LeftStickY = 0.0;
    private double gamepad2RightStickX = 0.0;
    private double gamepad2RightStickY = 0.0;
    private double gamepad2LeftTrigger = 0.0;
    private double gamepad2RightTrigger = 0.0;

    // Driver Input Variables
    boolean nextStep = false; // if nextStep is true, driver input will continue to the next step
    int driveFieldTiles = 0; // field tiles to be driven, indicated by drivers in init

    private void handleInput() {
        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;  // Whether or not the gamepad 1 a button was held
        boolean gamepad1BWasHeld = gamepad1BHeld;  // Whether or not the gamepad 1 b button was held
        boolean gamepad1XWasHeld = gamepad1XHeld;  // Whether or not the gamepad 1 x button was held
        boolean gamepad1YWasHeld = gamepad1YHeld;  // Whether or not the gamepad 1 y button was held
        boolean gamepad1LeftShoulderWasHeld = gamepad1LeftShoulderHeld;  // Whether or not the gamepad 1 left shoulder button was held
        boolean gamepad1RightShoulderWasHeld = gamepad1RightShoulderHeld;  // Whether or not the gamepad 1 left shoulder button was held
        boolean gamepad1LeftStickWasHeld = gamepad1LeftStickHeld;  // Whether or not the gamepad 1 left stick button was held
        boolean gamepad1RightStickWasHeld = gamepad1RightStickHeld;  // Whether or not the gamepad 1 right stick button was held
        boolean gamepad1DpadUpWasHeld = gamepad1DpadUpHeld;  // Whether or not the gamepad 1 dpad up button was held
        boolean gamepad1DpadDownWasHeld = gamepad1DpadDownHeld;  // Whether or not the gamepad 1 dpad down button was held
        boolean gamepad1DpadLeftWasHeld = gamepad1DpadLeftHeld;  // Whether or not the gamepad 1 dpad left button was held
        boolean gamepad1DpadRightWasHeld = gamepad1DpadRightHeld;  // Whether or not the gamepad 1 dpad right button was held

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1BHeld = gamepad1.b;
        gamepad1DpadUpHeld = gamepad1.dpad_up;
        gamepad1DpadDownHeld = gamepad1.dpad_down;
        gamepad1DpadLeftHeld = gamepad1.dpad_left;
        gamepad1DpadRightHeld = gamepad1.dpad_right;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;
        gamepad1DpadUpPressed = !gamepad1DpadUpWasHeld && gamepad1DpadUpHeld;
        gamepad1DpadDownPressed = !gamepad1DpadDownWasHeld && gamepad1DpadDownHeld;
        gamepad1DpadLeftPressed = !gamepad1DpadLeftWasHeld && gamepad1DpadLeftHeld;
        gamepad1DpadRightPressed = !gamepad1DpadRightWasHeld && gamepad1DpadRightHeld;

        // Cache previous gamepad 2 inputs
        boolean gamepad2AWasHeld = gamepad2AHeld;  // Whether or not the gamepad 2 a button was held
        boolean gamepad2BWasHeld = gamepad2BHeld;  // Whether or not the gamepad 2 b button was held
        boolean gamepad2DpadUpWasHeld = gamepad2DpadUpHeld;  // Whether or not the gamepad 2 dpad up button was held
        boolean gamepad2DpadDownWasHeld = gamepad2DpadDownHeld;  // Whether or not the gamepad 2 dpad down button was held
        boolean gamepad2DpadLeftWasHeld = gamepad2DpadLeftHeld;  // Whether or not the gamepad 2 dpad left button was held
        boolean gamepad2DpadRightWasHeld = gamepad2DpadRightHeld;  // Whether or not the gamepad 2 dpad right button was held

        // Get new values from the actual gamepad 2
        gamepad2AHeld = gamepad2.a;
        gamepad2BHeld = gamepad2.b;
        gamepad2DpadUpHeld = gamepad2.dpad_up;
        gamepad2DpadDownHeld = gamepad2.dpad_down;
        gamepad2DpadLeftHeld = gamepad2.dpad_left;
        gamepad2DpadRightHeld = gamepad2.dpad_right;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(STARTING_POSE);

        telemetry.addData("Status: ", "Obtaining user input...");

        while (!nextStep) {
            handleInput();
            if (gamepad1DpadRightPressed) driveFieldTiles++;
            else if (gamepad1DpadLeftPressed) driveFieldTiles--;
            else if (gamepad1APressed) nextStep = true;
            telemetry.addData("Obtaining ", "field tiles to drive = " + driveFieldTiles);
            telemetry.update();
        }

        telemetry.addData("Status: ", "Building Trajectories...");

// ROADRUNNER AUTONOMOUS TRAJECTORIES:

        Trajectory driveStraight = drive.trajectoryBuilder(STARTING_POSE)
                .lineTo(new Vector2d(STARTING_POSE.getX() + driveFieldTiles * 24, STARTING_POSE.getY()))
                .build();


// INITIALIZE HARDWARE:
        // Initialize intake motor
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

        // Initialize flywheel motor
        ringShooter = hardwareMap.get(DcMotorEx.class, "shoot");

        // Initialize Wobble Goal manipulation motors
        wobbleClaw = hardwareMap.get(Servo.class, "WGClaw");
        wobbleArm = hardwareMap.get(DcMotor.class, "WGArm");
        fingerServo = hardwareMap.get(Servo.class, "ringFinger");

        // Initialize ring elevator motor
        ringElevator = hardwareMap.get(DcMotor.class, "ringElevator");

        // Initialize motor and servo Positions
        // Set Motor Directions

        // Obtain encoder positions based on starting position and deltas
        // Obtain wobble arm position values
        ARM_STARTING_POSITION = wobbleArm.getCurrentPosition() + 20;  // Get the starting position of the arm based on the current position of the arm at init time, which is assumed.
        ARM_UP_POSITION = wobbleArm.getCurrentPosition() - ARM_UP_POSITION_DELTA;  // Get the up position of the arm with respect to the current position of the arm at init time, which is assumed.
        ARM_DOWN_POSITION = wobbleArm.getCurrentPosition() - ARM_DOWN_POSITION_DELTA;  // Get the down position of the arm with respect to the current position of the arm at init time, which is assumed.
        ARM_HOVER_POSITION = wobbleArm.getCurrentPosition() - ARM_HOVER_POSITION_DELTA;  // Get the hover position of the arm with respect to the current position of the arm at init time, which is assumed.

        // Obtain ring elevator position values
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2017;

        // Set flywheel PIDF coefficients
        ringShooter.setVelocityPIDFCoefficients(150, 7, 10, 0);

// WAIT FOR AUTONOMOUS TO BEGIN:
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        drive.followTrajectory(driveStraight);

    }
}