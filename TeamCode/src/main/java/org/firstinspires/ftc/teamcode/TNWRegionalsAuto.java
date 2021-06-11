package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(group = "drive")
public class TNWRegionalsAuto extends LinearOpMode {
    OpenCvCamera webcam;
    TNWRegionalsAuto.imageFeedPipeline pipeline = new imageFeedPipeline();  // used to be imageFeedPipeline, but required change to OpenCvPipeline

    // STARTER STACK DETECTION VALUES:
    // Starting position camera crop values to be set and used by OpenCV
    double[] RING_SCAN_CROP_PERCENTS = new double[4];  // X1, X2, Y1, and Y2, respectively

    // Starting position 1 camera crop values
    double[] RING_SCAN_CROP_PERCENTS_1_RED = {0.68, 0.87, 0.27, 0.47};  // X1, X2, Y1, and Y2, respectively for the first starting position on red
    double[] RING_SCAN_CROP_PERCENTS_2_RED = {0.01, 0.20, 0.3, 0.5};  // X1, X2, Y1, and Y2, respectively for the second starting position on red
    double[] RING_SCAN_CROP_PERCENTS_1_BLUE = {0.06, 0.25, 0.27, 0.47};  // X1, X2, Y1, and Y2, respectively for the first starting position on blue
    double[] RING_SCAN_CROP_PERCENTS_2_BLUE = {0.72, 0.91, 0.28, 0.48};  // X1, X2, Y1, and Y2, respectively for the second starting position on blue

    /*public static int LOWER_HSV_H = 0;
    public static int LOWER_HSV_S= 0;
    public static int LOWER_HSV_V = 0;
    public static int UPPER_HSV_H = 0;
    public static int UPPER_HSV_S = 0;
    public static int UPPER_HSV_V = 0;*/

    double ringImagePercent = 0.0;
    double oneRingPercentageMinimum = .04; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 1 ring scenario
    double fourRingPercentageMinimum = .15; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 4 ring scenario

    private static final int RING_COLOR_H_START = 13;  // 15
    private static final int RING_COLOR_S_START = 45;  // 51
    private static final int RING_COLOR_V_START = 0;  // 0
    private static final int RING_COLOR_H_END = 35;  // 32
    private static final int RING_COLOR_S_END = 92;  // 89
    private static final int RING_COLOR_V_END = 100;  // 100

    char autoCase = 'X';
    char autoCaseCapture = 'X';

    // ROADRUNNER VALUES:
    // Constant Roadrunner Pose Values
    // Starting poses
    final Pose2d STARTING_POSE_1 = new Pose2d(-63, -18, Math.toRadians(0)); // TODO: set this to allow more room for alliance partner and elements
    final Pose2d STARTING_POSE_2 = new Pose2d(-63, -(56 + 3.0/8), Math.toRadians(0)); // TODO: set these

    Pose2d SECONDARY_SHOOT_POSE_OUTER = new Pose2d(-2, -57, Math.toRadians(90) - Math.atan(/*X*/(72.0 - (-2)) / /*Y*/Math.abs(-57 - (-33.0))));
    Pose2d SECONDARY_SHOOT_POSE_INNER = new Pose2d(-2, -12, Math.toRadians(360) - (Math.toRadians(90) - Math.atan(/*X*/(72.0 - (-2)) / /*Y*/Math.abs((-12) - (-33.0)))));

    // Wobble goal poses
    final int DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING = 10; // inches

    // Alliance Partner Element pickup positions
    Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C; // used by the autonomous. to be set in initializeElementPositions() function

    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_RED = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_RED = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_RED = new Pose2d(-35-4, -55-4, Math.toRadians(330)); // TODO: set these
    final Pose2d PARTNER_SECOND_WOBBLE_GOAL_ALIGN_POSITION_RED = new Pose2d(PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_RED.getX() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_RED.getY() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_RED.getHeading()); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_ABSENT_RED = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_RED = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_ABSENT_RED = new Pose2d(-35-4, -55-4, Math.toRadians(330)); // TODO: set these
    final Pose2d PARTNER_SECOND_WOBBLE_GOAL_ALIGN_POSITION_ABSENT_RED = new Pose2d(PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_RED.getX() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_RED.getY() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_RED.getHeading()); // TODO: set these

    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_BLUE = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_BLUE = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_BLUE = new Pose2d(-35-4, -55-4, Math.toRadians(330)); // TODO: set these
    final Pose2d PARTNER_SECOND_WOBBLE_GOAL_ALIGN_POSITION_BLUE = new Pose2d(PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_BLUE.getX() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_BLUE.getY() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_BLUE.getHeading()); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_ABSENT_BLUE = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_BLUE = new Pose2d(-35, -52, Math.toRadians(15)); // TODO: set these
    final Pose2d PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_ABSENT_BLUE = new Pose2d(-35-4, -55-4, Math.toRadians(330)); // TODO: set these
    final Pose2d PARTNER_SECOND_WOBBLE_GOAL_ALIGN_POSITION_ABSENT_BLUE = new Pose2d(PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_BLUE.getX() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_BLUE.getY() + 12, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_BLUE.getHeading()); // TODO: set these

    Pose2d PARTNER_RINGS_POSITION = new Pose2d(-58, -28, Math.toRadians(90)); // used by the autonomous. to be set in initializeElementPositions() function

    //final Pose2d PARTNER_RINGS_POSITION_PRESENT = new Pose2d(-37, -23, 0); // TODO: set these
    //final Pose2d PARTNER_RINGS_POSITION_ABSENT = new Pose2d(0, 0, 0); // TODO: set these

    // Target zone positions
    Pose2d TARGET_ZONE_A1; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_A2; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_B1; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_B2; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_C1; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_C2; // used by the autonomous. to be set in initializeElementPositions() function

    final Pose2d TARGET_ZONE_A_RED_2 = new Pose2d(34, -57, Math.toRadians(30));
    final Pose2d TARGET_ZONE_B_RED_2 = new Pose2d(34, -50, Math.toRadians(270));
    final Pose2d TARGET_ZONE_C_RED_2 = new Pose2d(48, -50, Math.toRadians(180));

    final Pose2d TARGET_ZONE_A_BLUE_2 = new Pose2d(34, 57, Math.toRadians(0));
    final Pose2d TARGET_ZONE_B_BLUE_2 = new Pose2d(42, 50, Math.toRadians(90));
    final Pose2d TARGET_ZONE_C_BLUE_2 = new Pose2d(48, 50, Math.toRadians(210));

    final Pose2d TARGET_ZONE_A1_RED = new Pose2d(25, -42, Math.toRadians(90));
    final Pose2d TARGET_ZONE_A2_RED = new Pose2d(TARGET_ZONE_A1_RED.getX(), TARGET_ZONE_A1_RED.getY() + DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_A1_RED.getHeading());
    final Pose2d TARGET_ZONE_B1_RED = new Pose2d(25, -32, Math.toRadians(180));
    final Pose2d TARGET_ZONE_B2_RED = new Pose2d(TARGET_ZONE_B1_RED.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_B1_RED.getY() + 2, TARGET_ZONE_B1_RED.getHeading());
    final Pose2d TARGET_ZONE_C1_RED = new Pose2d(53, -50, Math.toRadians(180));
    final Pose2d TARGET_ZONE_C2_RED = new Pose2d(TARGET_ZONE_C1_RED.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_C1_RED.getY(), TARGET_ZONE_C1_RED.getHeading());
    final Pose2d TARGET_ZONE_A1_BLUE = new Pose2d(10, 44, Math.toRadians(270));
    final Pose2d TARGET_ZONE_A2_BLUE = new Pose2d(TARGET_ZONE_A1_BLUE.getX(), TARGET_ZONE_A1_BLUE.getY() + DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_A1_BLUE.getHeading());
    final Pose2d TARGET_ZONE_B1_BLUE = new Pose2d(24, 40, Math.toRadians(180));
    final Pose2d TARGET_ZONE_B2_BLUE = new Pose2d(TARGET_ZONE_B1_BLUE.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_B1_BLUE.getY() + 2, TARGET_ZONE_B1_BLUE.getHeading());
    final Pose2d TARGET_ZONE_C1_BLUE = new Pose2d(43, 54, Math.toRadians(225));
    final Pose2d TARGET_ZONE_C2_BLUE = new Pose2d(TARGET_ZONE_C1_BLUE.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_C1_BLUE.getY(), TARGET_ZONE_C1_BLUE.getHeading());


    // Starter stack related poses
    final Vector2d STARTER_STACK = new Vector2d(-24, -35);
    final Pose2d LONG_SHOT_POSE = new Pose2d(-40, -38, Math.toRadians(355)); // y = -36, Heading = 356
    // Power shot related poses
    //final Pose2d POWER_SHOT_SHOOT_1 = new Pose2d(-3, -3.5, Math.toRadians(356));
    //final double DISTANCE_BETWEEN_POWER_SHOTS = 8; // inches
    // Parking pose
    //final Pose2d PARKING_POSE = new Pose2d(12, -24, Math.toRadians(0));

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
    private DcMotorEx ringElevator;

    // MOTOR AND SERVO POSITION CONSTANTS:
    private static double highGoalTPS = 56.5 * 28; // 57.5  // Ticks per second of the shooter when active and aiming for the high goal
    private static double powerShotTPS = 50 * 28; // Ticks per second of the shooter when active and aiming for the power shots

    final int RING_SHOOT_TIME = 750; // ring scoring interval measured in milliseconds

    private static double CLAW_OPENED_POSITION = 0.0;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.48;  // The position of the claw when it is closed

    private static int ARM_DOWN_POSITION_DELTA = 422;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's down
    private static int ARM_UP_POSITION_DELTA = 222;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up
    private static int ARM_HOVER_POSITION_DELTA = 330;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up

    private static int ARM_DOWN_POSITION;  // The absolute position (in motor encoder units) of the arm's down position. Set on init
    private static int ARM_UP_POSITION;  // The absolute position (in motor encoder units) of the arm's up position. Set on init
    private static int ARM_HOVER_POSITION;  // The absolute position (in motor encoder units) of the arm's hover position. Set on init
    private static int ARM_STARTING_POSITION; // The absolute position (in motor encoder units) of the arm's starting position. Set on init

    private static int RING_ELEVATOR_UP_POSITION;  // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION;  // The position of the Ring Elevator when it is in the DOWN state
    private static double RING_ELEVATOR_POWER = 0.7;  // The power for the motor to use when running to its target position

    private static double RING_FINGER_IN_POSITION = 0.12;  // The position of the ring finger when it's in
    private static double RING_FINGER_OUT_POSITION = 0.50;  // The position of the ring finger when it's out

    private static double INTAKE_IN_POWER = -0.85;  // The power set to intake drive for collecting rings
    private static double INTAKE_OUT_POWER = INTAKE_IN_POWER * -1;  // The power set to intake drive for ejecting rings

    private long autoStartTime;

    // Gamepad inputs
    private boolean gamepad1AHeld = false;  // Whether or not the gamepad 1 a button is being held, handled by the handleInput function
    private boolean gamepad1APressed = false;  // Whether or not the gamepad 1 a button was JUST pressed, handled by the handleInput function
    private boolean gamepad1BHeld = false;  // Whether or not the gamepad 1 b button is being held, handled by the handleInput function
    private boolean gamepad1BPressed = false;  // // Whether or not the gamepad 1 b button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadLeftHeld = false;  // Whether or not the gamepad 1 dpad left button is being held, handled by the handleInput function
    private boolean gamepad1DpadRightHeld = false;  // Whether or not the gamepad 1 dpad right button is being held, handled by the handleInput function
    private boolean gamepad1DpadLeftPressed = false;  // Whether or not the gamepad 1 dpad left button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadRightPressed = false;  // Whether or not the gamepad 1 dpad right button was JUST pressed, handled by the handleInput function


    // Driver Input Variables
    boolean nextStep = false; // if nextStep is true, driver input will continue to the next step
    boolean blueAlliance = false; // are we on the blue alliance?  indicated by drivers in init
    byte startingPosition = 2; // starting position indicated by drivers in init
    boolean scoreAlliancePartnerWobble = true; // whether to collect and deliver alliance partner's wobble goal.  indicated by drivers in configuration
    boolean scoreAlliancePartnerRings = true;  // whether to collect and score alliance partner's preloaded rings.  indicated by drivers in configuration
    boolean deliverWobble = true; // whether to deliver preloaded wobble goal.  indicated by drivers in configuration
    boolean navigateToLaunchLine = true; // whether or not to park on the launch line at the end of the autonomous period.  indicated by drivers in configuration
    byte parkingLocation = 2; // field tiles from alliance wall to park indicated by drivers in configuration
    boolean alliancePartnerMoves = false;
    boolean scoreStarterStack = true;
    boolean buttonsReleased = true; // gamepad buttons are released before triggering queues

    SampleMecanumDrive drive;

    // FUNCTIONS:

    private void handleInput() {
        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;  // Whether or not the gamepad 1 a button was held
        boolean gamepad1BWasHeld = gamepad1BHeld;  // Whether or not the gamepad 1 b button was held
        boolean gamepad1DpadLeftWasHeld = gamepad1DpadLeftHeld;  // Whether or not the gamepad 1 dpad left button was held
        boolean gamepad1DpadRightWasHeld = gamepad1DpadRightHeld;  // Whether or not the gamepad 1 dpad right button was held

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1BHeld = gamepad1.b;
        gamepad1DpadLeftHeld = gamepad1.dpad_left;
        gamepad1DpadRightHeld = gamepad1.dpad_right;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1DpadLeftPressed = !gamepad1DpadLeftWasHeld && gamepad1DpadLeftHeld;
        gamepad1DpadRightPressed = !gamepad1DpadRightWasHeld && gamepad1DpadRightHeld;
    }

    private void initializeElementPositions() {
        for (int i = 0; i < 4; i++) {
            if (blueAlliance) {
                switch (startingPosition) {
                    case 1: RING_SCAN_CROP_PERCENTS[i] = RING_SCAN_CROP_PERCENTS_1_BLUE[i]; break;
                    case 2: RING_SCAN_CROP_PERCENTS[i] = RING_SCAN_CROP_PERCENTS_2_BLUE[i]; break;
                }
            }
            else {
                switch (startingPosition) {
                    case 1: RING_SCAN_CROP_PERCENTS[i] = RING_SCAN_CROP_PERCENTS_1_RED[i]; break;
                    case 2: RING_SCAN_CROP_PERCENTS[i] = RING_SCAN_CROP_PERCENTS_2_RED[i]; break;
                }
            }
        }
        if (blueAlliance) {
            TARGET_ZONE_A1 = TARGET_ZONE_A1_BLUE;
            TARGET_ZONE_B1 = TARGET_ZONE_B1_BLUE;
            TARGET_ZONE_C1 = TARGET_ZONE_C1_BLUE;
            TARGET_ZONE_A2 = TARGET_ZONE_A2_BLUE;
            TARGET_ZONE_B2 = TARGET_ZONE_B2_BLUE;
            TARGET_ZONE_C2 = TARGET_ZONE_C2_BLUE;
        }
        else {
            TARGET_ZONE_A1 = TARGET_ZONE_A1_RED;
            TARGET_ZONE_B1 = TARGET_ZONE_B1_RED;
            TARGET_ZONE_C1 = TARGET_ZONE_C1_RED;
            TARGET_ZONE_A2 = TARGET_ZONE_A2_RED;
            TARGET_ZONE_B2 = TARGET_ZONE_B2_RED;
            TARGET_ZONE_C2 = TARGET_ZONE_C2_RED;
        }
        if (alliancePartnerMoves) {
            if (blueAlliance) {
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_ABSENT_BLUE;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_BLUE;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_ABSENT_BLUE;
            }
            else {
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_ABSENT_RED;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_ABSENT_RED;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_ABSENT_RED;
            }
        }
        else {
            if (blueAlliance) {
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_BLUE;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_BLUE;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_BLUE;
            }
            else {
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A_RED;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_B_RED;
                PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C = PARTNER_WOBBLE_GOAL_PICKUP_POSITION_C_RED;
            }
        }
    }

    Pose2d selInvertPose(Pose2d inputPose) {
        if (blueAlliance) {
            return new Pose2d(inputPose.getX(), -inputPose.getY(), Math.toRadians(360) - inputPose.getHeading());
        }
        else { return inputPose; }
    }
    Pose2d selInvertPose(Pose2d inputPose, boolean universalHeading) {
        if (blueAlliance) {
            if (universalHeading) return new Pose2d(inputPose.getX(), -inputPose.getY(), inputPose.getHeading());
            else return new Pose2d(inputPose.getX(), -inputPose.getY(), Math.toRadians(360) - inputPose.getHeading());
        }
        else { return inputPose; }
    }
    Vector2d selInvertPose(Vector2d inputVector) {
        if (blueAlliance) {
            return new Vector2d(inputVector.getX(), -inputVector.getY());
        }
        else { return inputVector; }
    }
    double selInvertPose(double inputEndTangent) {
        if (blueAlliance) return Math.toRadians(360) - inputEndTangent;
        else return inputEndTangent;
    }

    private void stopMillis(long delay) {
        // Stops the robot for `delay` milliseconds
        // Assumes that a SampleMecanumDrive with a public mode variable and Mode enum is in scope
        drive.mode = SampleMecanumDrive.Mode.IDLE;
        long startTime = System.currentTimeMillis();

        drive.setMotorPowers(0, 0, 0, 0);

        while (System.currentTimeMillis() - startTime < delay) {  // Assumes we're running in an iterative OpMode where blocking is permitted
            drive.update();  // Keep track of PID, but IDLE Mode makes it try to stop the bot
        }

        drive.mode = SampleMecanumDrive.Mode.FOLLOW_TRAJECTORY;  // Return to path following
    }

    private void pause(long millis) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + millis) {
            stopMillis(1);
            ringShooter.setVelocity(highGoalTPS);
        }
    }
    private void pause(long millis, boolean flywheel) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + millis) {
            stopMillis(1);
            if (flywheel) ringShooter.setVelocity(highGoalTPS);
        }
    }
    private void pause(long millis, double TPS) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + millis) {
            stopMillis(1);
            ringShooter.setVelocity(TPS);
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // obtain alliance color info from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    blueAlliance = false;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    blueAlliance = true;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Red Alliance? (A for yes, B for no)");
                telemetry.update();
            }
        }
        // obtain starting line position from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    startingPosition = 1;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    startingPosition = 2;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Starting Position? (A for inner, B for outer)");
                telemetry.update();
            }
        }
        // obtain whether or not to score starter stack rings from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    scoreStarterStack = true;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    scoreStarterStack = false;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Score Starter Stack? (A for Yes, B for No)");
                telemetry.update();
            }
        }
        // obtain whether or not to navigate to launch line from drivers
        nextStep = false;
        buttonsReleased = false;
        while(!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) { navigateToLaunchLine = true; nextStep = true; }
                else if (gamepad1BPressed) { navigateToLaunchLine = false; nextStep = true; }
                telemetry.addData("Obtaining ", "Park? (A for Yes, B for No)");
                telemetry.update();
            }
        }
        // obtain parking location from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1DpadRightPressed) parkingLocation++;
                else if (gamepad1DpadLeftPressed) parkingLocation--;
                else if (gamepad1APressed) nextStep = true;
                telemetry.addData("Obtaining ", "field tiles from wall to park " + parkingLocation);
                telemetry.update();
            }
        }
        // obtain information about whether or not to deliver a wobble goal from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    deliverWobble = true;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    deliverWobble = false;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Deliver preloaded wobble goal? (A for yes, B for no)");
                telemetry.update();
            }
        }
        // obtain whether to score alliance partner rings from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    scoreAlliancePartnerRings = true;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    scoreAlliancePartnerRings = false;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Score alliance partner's rings? (A for Yes, B for No)");
                telemetry.update();
            }
        }
        // allow driver to review selected configuration
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Reviewing Selections (A to confirm)");
                if (blueAlliance) telemetry.addData("Alliance ", "BLUE");
                else telemetry.addData("Alliance ", "RED");
                if (startingPosition == 1) telemetry.addData("Starting Line ", "INNER");
                else telemetry.addData("Starting Line ", "OUTER");
                telemetry.addData("Field tiles from wall to park ", parkingLocation);
                if (deliverWobble) telemetry.addLine("Deliver Preloaded Wobble Goal");
                else telemetry.addLine("Don't Deliver Preloaded Wobble Goal");
                if (scoreAlliancePartnerRings) telemetry.addLine("Score Alliance Partner Rings");
                else telemetry.addLine("Don't Score Alliance Partner Rings");
                telemetry.update();
            }
        }

        drive.setPoseEstimate(selInvertPose(startingPosition == 1 ? STARTING_POSE_1 : STARTING_POSE_2));

        initializeElementPositions(); // keep this in init to initialize camera viewing borders

// INITIALIZE OPENCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

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
        ringElevator = hardwareMap.get(DcMotorEx.class, "ringElevator");

        // Initialize motor and servo Positions
        // Set Motor Directions
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set servo initialization positions
        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);

        // Apply motor power
        // Apply power and set mode for wobble arm motor
        wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
        wobbleArm.setPower(.5);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power and set mode for ring elevator motor
        ringElevator.setTargetPosition(ringElevator.getCurrentPosition());
        ringElevator.setPower(0.8);
        ringElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run mode

        // Obtain encoder positions based on starting position and deltas
        // Obtain wobble arm position values
        ARM_STARTING_POSITION = wobbleArm.getCurrentPosition() + 20;  // Get the starting position of the arm based on the current position of the arm at init time, which is assumed.
        ARM_UP_POSITION = wobbleArm.getCurrentPosition() - ARM_UP_POSITION_DELTA;  // Get the up position of the arm with respect to the current position of the arm at init time, which is assumed.
        ARM_DOWN_POSITION = wobbleArm.getCurrentPosition() - ARM_DOWN_POSITION_DELTA;  // Get the down position of the arm with respect to the current position of the arm at init time, which is assumed.
        ARM_HOVER_POSITION = wobbleArm.getCurrentPosition() - ARM_HOVER_POSITION_DELTA;  // Get the hover position of the arm with respect to the current position of the arm at init time, which is assumed.

        // Obtain ring elevator position values
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2000; // 2017

        // Set DcMotorEx PIDF coefficients
        ringShooter.setVelocityPIDFCoefficients(150, 7, 10, 0);
        ringElevator.setVelocityPIDFCoefficients(5, 3, 3, 0);


// WAIT FOR AUTONOMOUS TO BEGIN:
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        // store autonomous case
        switch (autoCase) {
            case 'A': autoCaseCapture = 'A'; break;
            case 'B': autoCaseCapture = 'B'; break;
            case 'C': autoCaseCapture = 'C'; break;
        }

        // set autonomous start time for pause at end
        autoStartTime = System.currentTimeMillis();

        // Build Roadrunner Trajectories

        Trajectory scorePreloadedRings = drive.trajectoryBuilder(selInvertPose(startingPosition == 1 ? STARTING_POSE_1 : STARTING_POSE_2)) // drive from the start pose to the starter stack and shoot preloaded rings
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        webcam.closeCameraDevice(); // turn off camera to save resources
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo out to make room to avoid collision with ring elevator
                        if (scoreStarterStack) ringShooter.setVelocity(57.75 * 28); // spin up ring shooter to score in high goal // using a different speed since we are farther back.
                        else ringShooter.setVelocity(54.5 * 28);
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to drop intake shield
                    }
                })
                .splineToConstantHeading(selInvertPose(scoreStarterStack ? startingPosition == 1 ? new Vector2d(STARTING_POSE_1.getX() + 0.1, STARTING_POSE_1.getY()) : new Vector2d(STARTING_POSE_2.getX() + 0.1, STARTING_POSE_2.getY())
                        : startingPosition == 1 ? new Vector2d(STARTING_POSE_1.getX() + 3, STARTING_POSE_1.getY() + 5) : new Vector2d(STARTING_POSE_2.getX() + 3, STARTING_POSE_2.getY() - 3)), Math.toRadians(0))
                .splineToSplineHeading(scoreStarterStack ? selInvertPose(new Pose2d(autoCaseCapture == 'A' ? LONG_SHOT_POSE.getX() + 26 : LONG_SHOT_POSE.getX(), LONG_SHOT_POSE.getY(), LONG_SHOT_POSE.getHeading()), true)
                        : startingPosition == 1 ? selInvertPose(new Pose2d(SECONDARY_SHOOT_POSE_INNER.getX(), SECONDARY_SHOOT_POSE_INNER.getY(), SECONDARY_SHOOT_POSE_INNER.getHeading() - selInvertPose(Math.toRadians(4))))
                        : selInvertPose(new Pose2d(SECONDARY_SHOOT_POSE_OUTER.getX(), SECONDARY_SHOOT_POSE_OUTER.getY(), SECONDARY_SHOOT_POSE_OUTER.getHeading() - selInvertPose(Math.toRadians(8)))), scoreStarterStack ? selInvertPose(Math.toRadians(startingPosition == 1 ? 270 : 90)) : Math.toRadians(0))
                .addTemporalMarker(.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(0); // turn off intake
                    }
                })
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500); // allow flywheel PID to adjust
                        for (int i = 1; i <= 4; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2, (autoCaseCapture == 'A' || !scoreStarterStack) ? highGoalTPS : 57.75 * 28);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2, (autoCaseCapture == 'A' || !scoreStarterStack) ? highGoalTPS : 57.75 * 28);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory shootStarterStackRings1 = drive.trajectoryBuilder(scorePreloadedRings.end()) // only used in auto cases B and C
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500, false);
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to collect starter stack rings
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 10, LONG_SHOT_POSE.getY(), Math.toRadians(356)), true), // slowly intake starter stack rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000); // allow rings time to completely enter elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move ring finger to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        ringShooter.setVelocity(highGoalTPS);
                        pause(1750); // allow time for ring elevator to raise
                        intakeDrive.setPower(0); // turn off intake
                        final int REPETITIONS = autoCaseCapture == 'B' ? 2 : 4;
                        for (int i = 1; i <= REPETITIONS; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory shootStarterStackRings2 = drive.trajectoryBuilder(shootStarterStackRings1.end()) // only used in auto cases B and C
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500, false);
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to collect starter stack rings
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 26, LONG_SHOT_POSE.getY(), Math.toRadians(356)), true), // slowly intake starter stack rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000); // allow rings time to completely enter elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move ring finger to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        ringShooter.setVelocity(highGoalTPS);
                        pause(1750); // allow time for ring elevator to raise
                        intakeDrive.setPower(0); // turn off intake
                        for (int i = 1; i <= 4; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory alignToCollectPartnerRings = drive.trajectoryBuilder(autoCaseCapture == 'A' ? scorePreloadedRings.end() : autoCaseCapture == 'B' ? shootStarterStackRings1.end() : shootStarterStackRings2.end())
                .lineToLinearHeading(selInvertPose(new Pose2d(PARTNER_RINGS_POSITION.getX(), startingPosition == 2 ? PARTNER_RINGS_POSITION.getY() - 25 : PARTNER_RINGS_POSITION.getY() + 10, Math.toRadians(startingPosition == 2 ? 90 : 270))),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI / 2, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // retract finger servo into robot to avoid accidental damage
                    }
                })
                .build();

        Trajectory collectAlliancePartnerRings = drive.trajectoryBuilder(alignToCollectPartnerRings.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_IN_POWER); // power on intake to collect alliance partner's rings
                    }
                })
                .lineToConstantHeading(selInvertPose(new Vector2d(PARTNER_RINGS_POSITION.getX(), startingPosition == 2 ? PARTNER_RINGS_POSITION.getY() + 10 : PARTNER_RINGS_POSITION.getY() - 10)),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500, false);
                    }
                })
                .build();

        Trajectory shootAlliancePartnerRings = drive.trajectoryBuilder(collectAlliancePartnerRings.end())
                .addTemporalMarker(1.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        intakeDrive.setPower(0); // turn off intake
                        ringShooter.setVelocity(highGoalTPS); // spin up flywheel to score in high goal
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 26, LONG_SHOT_POSE.getY(), LONG_SHOT_POSE.getHeading()), true))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000);
                        for (int i = 1; i <= 4; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory deliverWobbleGoal1 = drive.trajectoryBuilder(scoreStarterStack ?
                (scoreAlliancePartnerRings ? shootAlliancePartnerRings.end()
                        : autoCaseCapture == 'A' ? scorePreloadedRings.end()
                        : autoCaseCapture == 'B' ? shootStarterStackRings1.end()
                        : shootStarterStackRings2.end())
                : scorePreloadedRings.end())
                .addTemporalMarker(0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // deploy wobble arm
                    }
                })
                .addTemporalMarker(1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // retract finger servo to avoid accidental damage
                    }
                })
                .lineToLinearHeading(
                        scoreStarterStack ?
                                (autoCaseCapture == 'A' ? TARGET_ZONE_A1
                                        : autoCaseCapture == 'B' ? TARGET_ZONE_B1
                                        : TARGET_ZONE_C1)

                                : (blueAlliance ?
                                (autoCaseCapture == 'A' ? TARGET_ZONE_A_BLUE_2
                                        : autoCaseCapture == 'B' ? TARGET_ZONE_B_BLUE_2
                                        : TARGET_ZONE_C_BLUE_2)
                                :
                                (autoCaseCapture == 'A' ? TARGET_ZONE_A_RED_2
                                        : autoCaseCapture == 'B' ? TARGET_ZONE_B_RED_2
                                        : TARGET_ZONE_C_RED_2)))// because these poses are set manually, do not use selInvertPose on them.
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        if (!scoreStarterStack) {
                            if (autoCaseCapture == 'A') pause(1000, false);
                            else if (autoCaseCapture == 'B') pause(1000, false);
                            else pause(1000, false);
                        }
                        else {
                            pause(1000, false);
                        }
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION); // open wobble claw to release wobble goal
                        pause(250, false);
                        wobbleArm.setTargetPosition(ARM_STARTING_POSITION); // completely retract wobble arm
                        pause(1000, false);
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);
                    }
                })
                .build();

        Trajectory backFromTargetZone1 = drive.trajectoryBuilder(deliverWobbleGoal1.end())
                .lineToLinearHeading(autoCaseCapture == 'A' ? new Pose2d(TARGET_ZONE_A1.getX(), blueAlliance ? TARGET_ZONE_A1.getY() - 14 : TARGET_ZONE_A1.getY() + 14, TARGET_ZONE_A1.getHeading() - Math.toRadians(45))
                        : autoCaseCapture == 'B' ? new Pose2d(TARGET_ZONE_B1.getX() - 14, TARGET_ZONE_B1.getY(), TARGET_ZONE_B1.getHeading() - Math.toRadians(45))
                        : selInvertPose(new Pose2d(TARGET_ZONE_C1.getX() - 14, -56 + (parkingLocation - 1) * 24, Math.toRadians(0))))
                .addTemporalMarker(.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);
                    }
                })
                .build();

        Trajectory alignToPark = drive.trajectoryBuilder(deliverWobbleGoal1.end())
                .lineToLinearHeading(selInvertPose(autoCaseCapture == 'A' ? (new Pose2d(40, -57 + ((parkingLocation != 1 ? parkingLocation : (autoCaseCapture != 'A') ? 1 : 2) - 1) * 24, Math.toRadians(0)))
                        : (new Pose2d(40, -57 + ((parkingLocation != 1 ? parkingLocation : (autoCaseCapture != 'A') ? 1 : 2) - 1) * 24, Math.toRadians(0)))))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);
                        wobbleArm.setTargetPosition(ARM_STARTING_POSITION);
                    }
                })
                .build();

        Trajectory park = drive.trajectoryBuilder(
                deliverWobble && scoreStarterStack && (autoCaseCapture == 'A' || !scoreAlliancePartnerRings) ? backFromTargetZone1.end()
                        : scoreStarterStack ? (scoreAlliancePartnerRings ? shootAlliancePartnerRings.end()
                        : autoCaseCapture == 'C' ? shootStarterStackRings2.end()
                        : autoCaseCapture == 'B' ? shootStarterStackRings1.end()
                        : scorePreloadedRings.end())
                        : alignToPark.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        if (scoreStarterStack) wobbleArm.setTargetPosition(ARM_STARTING_POSITION);
                        ringShooter.setPower(0);
                    }
                })
                .addTemporalMarker(1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(12, -57 + ((parkingLocation != 1 ? parkingLocation : (autoCaseCapture != 'A') ? 1 : 2) - 1) * 24, Math.toRadians(0))))
                .build();


        drive.followTrajectory(scorePreloadedRings);
        if (scoreStarterStack) {
            if (autoCaseCapture != 'A') {
                drive.followTrajectory(shootStarterStackRings1);
                if (autoCaseCapture == 'C') {
                    drive.followTrajectory(shootStarterStackRings2);
                }
            }
        }
        if (scoreAlliancePartnerRings && (autoCaseCapture != 'C' || !deliverWobble) && scoreStarterStack) {
            drive.followTrajectory(alignToCollectPartnerRings);
            drive.followTrajectory(collectAlliancePartnerRings);
            if (autoCaseCapture != 'C') drive.followTrajectory(shootAlliancePartnerRings);
        }
        if (deliverWobble && (autoCaseCapture != 'B' || !scoreAlliancePartnerRings || !scoreStarterStack)) {
            drive.followTrajectory(deliverWobbleGoal1);
            if (scoreStarterStack) drive.followTrajectory(backFromTargetZone1);
        }
        if (!scoreStarterStack) {
            drive.followTrajectory(alignToPark);
        }
        if (navigateToLaunchLine && (autoCaseCapture != 'C' || !scoreAlliancePartnerRings || !scoreStarterStack)) {
            drive.followTrajectory(park);
        }
        wobbleArm.setTargetPosition(ARM_STARTING_POSITION);
        intakeDrive.setPower(0);
        ringShooter.setPower(0);
        pause(autoStartTime + 29900 - System.currentTimeMillis(), false);
    }
    class imageFeedPipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        Mat imageCrop = new Mat();
        Mat HSVImage = new Mat();
        Mat ringMask = new Mat();
        int ringPixels = 0;
        double[] pixel;
        double[] setpixel;
        Point setpixelPoint;
        double redValue, greenValue, blueValue;
        final int resolutionTuner = 5; // One pixel sampled every # pixels.  Raise for speed, lower for reliability.
        final int RED_VALUE_MIN = 100;
        final double ORANGE_GB_LOW_THRESHOLD = 1.1; // 1.5 on old webcam
        final double ORANGE_RG_LOW_THRESHOLD = 1.25; // possibly increase for new webcam?
        final double ORANGE_RB_LOW_THRESHOLD = 5.0;
        public int totalPixels = 0;


        @Override
        public Mat processFrame(Mat imageFeed) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            // OLD VISION CODE
            final int RING_SECTION_CROP_Y1 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENTS[2]);
            final int RING_SECTION_CROP_Y2 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENTS[3]);
            final int RING_SECTION_CROP_X1 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENTS[0]);
            final int RING_SECTION_CROP_X2 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENTS[1]);
            final Scalar ringVisualizeColor = new Scalar(0.0d, 255.0d, 0.0d);
            /*

            setpixelPoint = new Point(0, 0);

            totalPixels = ((RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) * (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1) / resolutionTuner);

            ringPixels = 0;
            for(int x = RING_SECTION_CROP_X1; x <= RING_SECTION_CROP_X2; x += resolutionTuner) {
                for (int y = RING_SECTION_CROP_Y1; y <= RING_SECTION_CROP_Y2; y += resolutionTuner) {
                    pixel = imageFeed.get(y, x);
                    redValue = pixel[0];
                    greenValue = pixel[1];
                    blueValue = pixel[2];
                    if (/*redValue >= blueValue * ORANGE_RB_LOW_THRESHOLD && *//*redValue >= greenValue * ORANGE_RG_LOW_THRESHOLD && greenValue >= blueValue * ORANGE_GB_LOW_THRESHOLD) {
                        ringPixels++;
                        // imageFeed.set(y, x, setpixel)
                        setpixelPoint.x = (int) x;
                        setpixelPoint.y = (int) y;
                        Imgproc.circle(imageFeed, setpixelPoint, resolutionTuner / 2, ringVisualizeColor, Imgproc.FILLED);
                    }
                }
            }

            Imgproc.rectangle(
                    imageFeed,
                    new Point(
                            RING_SECTION_CROP_X1,
                            RING_SECTION_CROP_Y1),
                    new Point(
                            RING_SECTION_CROP_X2,
                            RING_SECTION_CROP_Y2),
                    new Scalar(0, 255, 0), 4);


            ringImagePercent = (double) ringPixels / ((double) totalPixels / resolutionTuner);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            /*
             * Send some stats to the telemetry
             *//*

            char autoCase_ = 'X';

            if (ringImagePercent >= oneRingPercentageMinimum && ringImagePercent < fourRingPercentageMinimum) { autoCase = 'B'; }
            else if (ringImagePercent >= fourRingPercentageMinimum) { autoCase = 'C'; }
            else {autoCase = 'A'; }

            telemetry.addData("Auto Case: ", (char) (autoCase));
            telemetry.addData("Ring Percentage: ", ringImagePercent);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Ring-Colored Pixels: ", totalPixels);
            telemetry.addData("Ring crop region X1", RING_SCAN_CROP_PERCENTS[0]);
            telemetry.addData("Ring crop region Y1", RING_SCAN_CROP_PERCENTS[2]);
            telemetry.addData("Ring crop region X2", RING_SCAN_CROP_PERCENTS[1]);
            telemetry.addData("Ring crop region Y2", RING_SCAN_CROP_PERCENTS[3]);
            telemetry.addData("Total pixels", totalPixels);
            telemetry.update();

            return imageFeed;*/
            // NEW VISION CODE:
            /*final Scalar LOWER_HSV = new Scalar(0, 0, 0);
            final Scalar UPPER_HSV = new Scalar(180, 255, 160);*/
            /*final Scalar LOWER_HSV = new Scalar(LOWER_HSV_H, LOWER_HSV_S, LOWER_HSV_V);
            final Scalar UPPER_HSV = new Scalar(UPPER_HSV_H, UPPER_HSV_S, UPPER_HSV_V);*/
            final Scalar LOWER_HSV = new Scalar(80, 60, 0);
            final Scalar UPPER_HSV = new Scalar(110, 255, 255);
            Imgproc.cvtColor(imageFeed, HSVImage, Imgproc.COLOR_BGR2HSV);
            Core.inRange(HSVImage, LOWER_HSV, UPPER_HSV, ringMask);
            //Core.bitwise_not(ringMask, ringMask);

            //HSVImage.release();  // Don't leak memory!

            ringMask = ringMask.submat(new Rect(new Point(RING_SECTION_CROP_X1, RING_SECTION_CROP_Y1), new Point(RING_SECTION_CROP_X2, RING_SECTION_CROP_Y2)));
            ringPixels = Core.countNonZero(ringMask);
            totalPixels = ((RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) * (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1));
            ringImagePercent = (double) ringPixels / ((double) totalPixels);

            //ringMask.release();

            Imgproc.rectangle(
                    imageFeed,
                    new Point(
                            RING_SECTION_CROP_X1,
                            RING_SECTION_CROP_Y1),
                    new Point(
                            RING_SECTION_CROP_X2,
                            RING_SECTION_CROP_Y2),
                    new Scalar(0, 255, 0), 4);

            char autoCase_ = 'X';

            if (ringImagePercent >= oneRingPercentageMinimum && ringImagePercent < fourRingPercentageMinimum) { autoCase = 'B'; }
            else if (ringImagePercent >= fourRingPercentageMinimum) { autoCase = 'C'; }
            else {autoCase = 'A'; }

            telemetry.addData("Auto Case: ", (char) (autoCase));
            telemetry.addData("Ring Percentage: ", ringImagePercent);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Ring-Colored Pixels: ", totalPixels);
            telemetry.addData("Ring crop region X1", RING_SCAN_CROP_PERCENTS[0]);
            telemetry.addData("Ring crop region Y1", RING_SCAN_CROP_PERCENTS[2]);
            telemetry.addData("Ring crop region X2", RING_SCAN_CROP_PERCENTS[1]);
            telemetry.addData("Ring crop region Y2", RING_SCAN_CROP_PERCENTS[3]);
            telemetry.addData("Total pixels", totalPixels);
            telemetry.update();

            return ringMask;
        }

        /*public int getRingPixels() {
            return ringPixels;
        }*/

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}

