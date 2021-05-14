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
public class TNWRegionalsAuto extends LinearOpMode {
    OpenCvCamera webcam;
    TNWRegionalsAuto.imageFeedPipeline pipeline = new imageFeedPipeline();  // used to be imageFeedPipeline, but required change to OpenCvPipeline

// STARTER STACK DETECTION VALUES:
    // Starting position camera crop values to be set and used by OpenCV
    public static double[] RING_SCAN_CROP_PERCENTS = new double[4];  // X1, X2, Y1, and Y2, respectively

    // Starting position 1 camera crop values
    public static double[] RING_SCAN_CROP_PERCENTS_1_RED = {0.4, 0.8, 0.1, 0.6};  // X1, X2, Y1, and Y2, respectively for the first starting position on red
    public static double[] RING_SCAN_CROP_PERCENTS_2_RED = {0.4, 0.8, 0.1, 0.6};  // X1, X2, Y1, and Y2, respectively for the second starting position on red TODO: set these
    public static double[] RING_SCAN_CROP_PERCENTS_1_BLUE = {0.4, 0.8, 0.1, 0.6};  // X1, X2, Y1, and Y2, respectively for the first starting position on blue TODO: set these
    public static double[] RING_SCAN_CROP_PERCENTS_2_BLUE = {0.0, 0.0, 0.0, 0.0};  // X1, X2, Y1, and Y2, respectively for the second starting position on blue TODO: set these

    double ringImagePercent = 0.0;
    double oneRingPercentageMinimum = .01; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 1 ring scenario
    double fourRingPercentageMinimum = .10; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 4 ring scenario

    private static final int RING_COLOR_H_START = 13;  // 15
    private static final int RING_COLOR_S_START = 45;  // 51
    private static final int RING_COLOR_V_START = 0;  // 0
    private static final int RING_COLOR_H_END = 35;  // 32
    private static final int RING_COLOR_S_END = 92;  // 89
    private static final int RING_COLOR_V_END = 100;  // 100

    char autoCase = 'X';


// ROADRUNNER VALUES:
    // Constant Roadrunner Pose Values
    // Starting poses
    final Pose2d STARTING_POSE_1 = new Pose2d(-63, -32, Math.toRadians(0)); // TODO: set this to allow more room for alliance partner and elements
    final Pose2d STARTING_POSE_2 = new Pose2d(-63, -52, 0); // TODO: set these

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

    Pose2d PARTNER_RINGS_POSITION; // used by the autonomous. to be set in initializeElementPositions() function

    final Pose2d PARTNER_RINGS_POSITION_PRESENT = new Pose2d(-37, -23, 0); // TODO: set these
    final Pose2d PARTNER_RINGS_POSITION_ABSENT = new Pose2d(0, 0, 0); // TODO: set these

    // Target zone positions
    Pose2d TARGET_ZONE_A1; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_A2; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_B1; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_B2; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_C1; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d TARGET_ZONE_C2; // used by the autonomous. to be set in initializeElementPositions() function

    final Pose2d TARGET_ZONE_A1_RED = new Pose2d(25, -49, Math.toRadians(90));
    final Pose2d TARGET_ZONE_A2_RED = new Pose2d(TARGET_ZONE_A1_RED.getX(), TARGET_ZONE_A1_RED.getY() + DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_A1_RED.getHeading());
    final Pose2d TARGET_ZONE_B1_RED = new Pose2d(30, -34, Math.toRadians(180));
    final Pose2d TARGET_ZONE_B2_RED = new Pose2d(TARGET_ZONE_B1_RED.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_B1_RED.getY() + 2, TARGET_ZONE_B1_RED.getHeading());
    final Pose2d TARGET_ZONE_C1_RED = new Pose2d(53, -50, Math.toRadians(180));
    final Pose2d TARGET_ZONE_C2_RED = new Pose2d(TARGET_ZONE_C1_RED.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_C1_RED.getY(), TARGET_ZONE_C1_RED.getHeading());
    final Pose2d TARGET_ZONE_A1_BLUE = new Pose2d(24, -51, Math.toRadians(90));
    final Pose2d TARGET_ZONE_A2_BLUE = new Pose2d(TARGET_ZONE_A1_BLUE.getX(), TARGET_ZONE_A1_BLUE.getY() + DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_A1_BLUE.getHeading());
    final Pose2d TARGET_ZONE_B1_BLUE = new Pose2d(30, -34, Math.toRadians(180));
    final Pose2d TARGET_ZONE_B2_BLUE = new Pose2d(TARGET_ZONE_B1_BLUE.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_B1_BLUE.getY() + 2, TARGET_ZONE_B1_BLUE.getHeading());
    final Pose2d TARGET_ZONE_C1_BLUE = new Pose2d(53, -50, Math.toRadians(180));
    final Pose2d TARGET_ZONE_C2_BLUE = new Pose2d(TARGET_ZONE_C1_BLUE.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_C1_BLUE.getY(), TARGET_ZONE_C1_BLUE.getHeading());


    // Starter stack related poses
    final Vector2d STARTER_STACK = new Vector2d(-24, -35);
    final Pose2d LONG_SHOT_POSE = new Pose2d(-37, -36, Math.toRadians(356));
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
    private static double highGoalTPS = 57 * 28; // 57.5  // Ticks per second of the shooter when active and aiming for the high goal
    private static double powerShotTPS = 50 * 28; // Ticks per second of the shooter when active and aiming for the power shots

    private static double CLAW_OPENED_POSITION = 0.24;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.80;  // The position of the claw when it is closed

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

    private static double RING_FINGER_IN_POSITION = 0.05;  // The position of the ring finger when it's in
    private static double RING_FINGER_OUT_POSITION = 0.4;  // The position of the ring finger when it's out

    private static double INTAKE_IN_POWER = -1;  // The power set to intake drive for collecting rings
    private static double INTAKE_OUT_POWER = INTAKE_IN_POWER * -1;  // The power set to intake drive for ejecting rings

    // Gamepad inputs
    private boolean gamepad1AHeld = false;  // Whether or not the gamepad 1 a button is being held, handled by the handleInput function
    private boolean gamepad1APressed = false;  // Whether or not the gamepad 1 a button was JUST pressed, handled by the handleInput function
    private boolean gamepad1BHeld = false;  // Whether or not the gamepad 1 b button is being held, handled by the handleInput function
    private boolean gamepad1BPressed = false;  // // Whether or not the gamepad 1 b button was JUST pressed, handled by the handleInput function

    // Driver Input Variables
    boolean nextStep = false; // if nextStep is true, driver input will continue to the next step
    boolean blueAlliance = false; // are we on the blue alliance?  indicated by drivers in init
    byte startingPosition = 2; // starting position indicated by drivers in init
    boolean scoreAlliancePartnerWobble = true; // whether to collect and deliver alliance partner's wobble goal.  indicated by drivers in configuration
    boolean scoreAlliancePartnerRings = true;  // whether to collect and score alliance partner's preloaded rings.  indicated by drivers in configuration
    byte parkingLocation = 2; // field tiles from alliance wall to park indicated by drivers in configuration
    boolean alliancePartnerMoves = false;
    boolean buttonsReleased = true; // gamepad buttons are released before triggering queues

    // FUNCTIONS:

    private void handleInput() {
        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;  // Whether or not the gamepad 1 a button was held
        boolean gamepad1BWasHeld = gamepad1BHeld;  // Whether or not the gamepad 1 b button was held

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1BHeld = gamepad1.b;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;  // The button was just pressed if our previous value was false, and this one was true
    }

    private void initializeElementPositions() {
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
            PARTNER_RINGS_POSITION = PARTNER_RINGS_POSITION_ABSENT;
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
            PARTNER_RINGS_POSITION = PARTNER_RINGS_POSITION_PRESENT;
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
        if (blueAlliance) return 360 - inputEndTangent;
        else return inputEndTangent;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        drive.setPoseEstimate(startingPosition == 1 ? STARTING_POSE_1 : STARTING_POSE_2);

        // Send configuration information to drivers for verification
        if (blueAlliance) telemetry.addData("Alliance ", "BLUE");
        else telemetry.addData("Alliance ", "RED");
        if (startingPosition == 1) telemetry.addData("Starting Line ", "INNER");
        else telemetry.addData("Starting Line ", "OUTER");


        telemetry.addData("Status: ", "Building Trajectories...");


// ROADRUNNER AUTONOMOUS TRAJECTORIES:

        initializeElementPositions(); // get bearings based on driver input data

        // Auto Case A Trajectories

        Trajectory alignToScorePreloadedRings = drive.trajectoryBuilder(selInvertPose(startingPosition == 1 ? STARTING_POSE_1 : STARTING_POSE_2)) // drive from the start pose to the starter stack and shoot preloaded rings
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        webcam.closeCameraDevice(); // turn off camera to save resources
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo out to make room to avoid collision with ring elevator
                        ringShooter.setVelocity(highGoalTPS); // spin up ring shooter to score in high goal
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to drop intake shield
                    }
                })
                .lineToLinearHeading(selInvertPose(LONG_SHOT_POSE, true))
                // a series of temporal markers is preferred over a looped sequence with pauses to preserve roadrunner PID accuracy
                .addTemporalMarker(.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(0); // turn off intake
                    }
                })
                .build();

        Trajectory scorePreloadedRings = drive.trajectoryBuilder(selInvertPose(alignToScorePreloadedRings.end()))
                .lineTo(new Vector2d(LONG_SHOT_POSE.getX() - 8, LONG_SHOT_POSE.getY()),
                        SampleMecanumDrive.getVelocityConstraint(2.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, new MarkerCallback() { // at 2 seconds, shoot rings
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(.4, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(.8, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(1.2, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(1.6, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(2.0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(2.4, new MarkerCallback() { // shoot 4 times in case a ring was jammed
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(2.8, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory deliverWobbleGoal1A = drive.trajectoryBuilder(selInvertPose(scorePreloadedRings.end(), true))
                .addTemporalMarker(1, new MarkerCallback() {
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
                .lineToLinearHeading(TARGET_ZONE_A1) // because this pose is set manually, do not use selInvertPose on this pose.
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION); // open wobble claw to release wobble goal
                        wobbleArm.setTargetPosition(ARM_STARTING_POSITION); // completely retract wobble arm
                    }
                })
                .build();

        Trajectory backFromTargetZone1A = drive.trajectoryBuilder(deliverWobbleGoal1A.end())
                .lineToLinearHeading(selInvertPose(new Pose2d(TARGET_ZONE_A1.getX(), TARGET_ZONE_A1.getY() + 14, TARGET_ZONE_A1.getHeading() - Math.toRadians(45))))
                .addTemporalMarker(.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);
                    }
                })
                .build();

      /*  Trajectory collectPartnerWobbleGoal = drive.trajectoryBuilder(backFromTargetZone1A.end())
                .lineToLinearHeading(PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION); // close wobble claw to capture alliance partner's wobble goal
                    }
                })
                .build();

        Trajectory backFromAlliancePartnerWobbleGoal = drive.trajectoryBuilder(collectPartnerWobbleGoal.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_HOVER_POSITION);
                    }
                })
                .lineToLinearHeading(new Pose2d(PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A.getX() + 5, PARTNER_WOBBLE_GOAL_PICKUP_POSITION_A.getY(), Math.toRadians(150)))
                .build();*/

        Trajectory alignToCollectPartnerRings = drive.trajectoryBuilder(selInvertPose(backFromTargetZone1A.end()))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_IN_POWER); // power on intake to collect alliance partner's rings
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(PARTNER_RINGS_POSITION.getX(), PARTNER_RINGS_POSITION.getY() - 8, Math.toRadians(150))))
                .build();

        Trajectory collectAlliancePartnerRings = drive.trajectoryBuilder(selInvertPose(alignToCollectPartnerRings.end()))
                .lineToConstantHeading(selInvertPose(new Vector2d(PARTNER_RINGS_POSITION.getX(), PARTNER_RINGS_POSITION.getY() + 15)),
                        SampleMecanumDrive.getVelocityConstraint(16, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory alignToScoreAlliancePartnerRings = drive.trajectoryBuilder(selInvertPose(collectAlliancePartnerRings.end()))
                .addTemporalMarker(1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        intakeDrive.setPower(0); // turn off intake
                        ringShooter.setVelocity(56.5 * 28); // spin up flywheel to score in high goal
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 32, LONG_SHOT_POSE.getY(), LONG_SHOT_POSE.getHeading()), true))
                .build();

        Trajectory scoreAlliancePartnerRings = drive.trajectoryBuilder(selInvertPose(alignToScoreAlliancePartnerRings.end(), true))
                .lineTo(selInvertPose(new Vector2d(LONG_SHOT_POSE.getX() + 24, LONG_SHOT_POSE.getY())),
                        SampleMecanumDrive.getVelocityConstraint(2.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, new MarkerCallback() { // at 2 seconds, shoot rings
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(.4, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(.8, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(1.2, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(1.6, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(2.0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(2.4, new MarkerCallback() { // shoot 4 times in case a ring was jammed
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(2.8, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory parkA = drive.trajectoryBuilder(selInvertPose(scoreAlliancePartnerRings.end(), true))
                .lineToLinearHeading(selInvertPose(new Pose2d(12, -56 + (parkingLocation - 1) * 24, Math.toRadians(0))))
                .build();



        Trajectory shootStarterStackRings1 = drive.trajectoryBuilder(selInvertPose(scorePreloadedRings.end(), true)) // only used in auto cases B and C
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to collect starter stack rings
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 10, STARTER_STACK.getY(), Math.toRadians(356)), true), // slowly intake starter stack rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(3, new MarkerCallback() { // after 3 seconds, trigger shoot sequence
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move ring finger to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        ringShooter.setVelocity(highGoalTPS);
                    }
                })
                .addTemporalMarker(4, new MarkerCallback() { // shoot rings
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(4.25, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(4.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(4.75, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION);
                    }
                })
                .build();

        Trajectory shootStarterStackRings2 = drive.trajectoryBuilder(selInvertPose(scorePreloadedRings.end(), true)) // only used in auto case C
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to collect starter stack rings
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 20, STARTER_STACK.getY(), Math.toRadians(356)), true), // slowly intake starter stack rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(3, new MarkerCallback() { // after 3 seconds, trigger shoot sequence
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move ring finger to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        ringShooter.setVelocity(highGoalTPS);
                    }
                })
                .addTemporalMarker(4, new MarkerCallback() { // shoot rings
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(4.25, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                    }
                })
                .addTemporalMarker(4.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                    }
                })
                .addTemporalMarker(4.75, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION);
                    }
                })
                .build();


// INITIALIZE OPENCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
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

        drive.followTrajectory(alignToScorePreloadedRings);
        drive.followTrajectory(scorePreloadedRings);

        if (autoCase == 'A') {
            drive.followTrajectory(deliverWobbleGoal1A);
            drive.followTrajectory(backFromTargetZone1A);
            drive.followTrajectory(alignToCollectPartnerRings);
            drive.followTrajectory(collectAlliancePartnerRings);
            drive.followTrajectory(alignToScoreAlliancePartnerRings);
            drive.followTrajectory(scoreAlliancePartnerRings);
            drive.followTrajectory(parkA);
        }


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
            Mat imageHSV = new Mat();
            Mat ringMask = new Mat();
            int ringPixels = 0;
            double[] pixel;
            double[] setpixel;
            Point setpixelPoint;
            double redValue, greenValue, blueValue;
            final int resolutionTuner = 5; // One pixel sampled every # pixels.  Raise for speed, lower for reliability.
            final int RED_VALUE_MIN = 100;
            final double ORANGE_GB_LOW_THRESHOLD = 1.5;
            final double ORANGE_RG_LOW_THRESHOLD = 1.25;
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

                final int RING_SECTION_CROP_Y1 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENTS[2]);
                final int RING_SECTION_CROP_Y2 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENTS[3]);
                final int RING_SECTION_CROP_X1 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENTS[0]);
                final int RING_SECTION_CROP_X2 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENTS[1]);
                final Scalar ringVisualizeColor = new Scalar(0.0d, 255.0d, 0.0d);

                setpixelPoint = new Point(0, 0);

                totalPixels = ((RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) * (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1) / resolutionTuner);

                ringPixels = 0;
                for(int x = RING_SECTION_CROP_X1; x <= RING_SECTION_CROP_X2; x += resolutionTuner) {
                    for (int y = RING_SECTION_CROP_Y1; y <= RING_SECTION_CROP_Y2; y += resolutionTuner) {
                        pixel = imageFeed.get(y, x);
                        redValue = pixel[0];
                        greenValue = pixel[1];
                        blueValue = pixel[2];
                        if (/*redValue >= blueValue * ORANGE_RB_LOW_THRESHOLD && */redValue >= greenValue * ORANGE_RG_LOW_THRESHOLD && greenValue >= blueValue * ORANGE_GB_LOW_THRESHOLD) {
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
                  */

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

                return imageFeed;
            }

            public int getRingPixels() {
                return ringPixels;
            }

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

