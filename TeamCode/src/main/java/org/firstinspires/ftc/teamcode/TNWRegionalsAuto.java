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
    TNWRegionalsAuto.imageFeedPipeline pipeline = new TNWRegionalsAuto.imageFeedPipeline();

// STARTER STACK DETECTION VALUES:
    public static double RING_SCAN_CROP_PERCENT_X1 = 0.4;  // 0.0
    public static double RING_SCAN_CROP_PERCENT_X2 = 0.8;  // 1.0
    public static double RING_SCAN_CROP_PERCENT_Y1 = 0.1;  // .49
    public static double RING_SCAN_CROP_PERCENT_Y2 = 0.6;  // .75

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
    // Starting pose
    final Pose2d STARTING_POSE = new Pose2d(-63, -32, Math.toRadians(0));

    // Target zone poses
    final int DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING = 10; // inches
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_A = new Pose2d(-35, -52, Math.toRadians(15));
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_B = new Pose2d(-35, -52, Math.toRadians(15));
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_C = new Pose2d(-35-4, -55-4, Math.toRadians(330));
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
    private DcMotorEx ringElevator;

    // MOTOR AND SERVO POSITION CONSTANTS:
    private static double highGoalTPS = 57.5 * 28;  // Ticks per second of the shooter when active and aiming for the high goal
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
    int driveFieldTiles = 0; // field tiles to be driven, indicated by drivers in init
    boolean blueAlliance = false;
    boolean buttonsReleased = true;

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

    Pose2d selInvertPose(Pose2d inputPose) {
        if (blueAlliance) {
            return new Pose2d(inputPose.getX(), -inputPose.getY(), Math.toRadians(360) - inputPose.getHeading());
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // obtain alliance color info from drivers
        nextStep = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) { blueAlliance = false; nextStep = true; }
                else if (gamepad1BPressed) { blueAlliance = true; nextStep = true; }
                telemetry.addData("Obtaining ", "Red Alliance? (A for yes, B for no)");
                telemetry.update();
            }
        }

        drive.setPoseEstimate(STARTING_POSE);

        telemetry.addData("Status: ", "Building Trajectories...");

// ROADRUNNER AUTONOMOUS TRAJECTORIES:

        // Common Trajectories

        Trajectory shootStartingRings = drive.trajectoryBuilder(selInvertPose(STARTING_POSE)) // drive from the start pose to the starter stack and shoot preloaded rings
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo out to make room to avoid collision with ring elevator
                        ringShooter.setVelocity(highGoalTPS); // spin up ring shooter to score in high goal
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(STARTER_STACK.getX(), STARTER_STACK.getY(), Math.toRadians(356))))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                       for (int i = 1; i <= 3; i++) {
                           fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                           pause(250);
                           fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                           pause(250);
                       }
                    }
                })
                .build();

        // Autonomous Case A Trajectories:


        // Autonomous Case B Trajectories:


        // Autonomous Case C Trajectories:


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
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2017;

        // Set DcMotorEx PIDF coefficients
        ringShooter.setVelocityPIDFCoefficients(150, 7, 10, 0);
        ringElevator.setVelocityPIDFCoefficients(5, 3, 3, 0);


// WAIT FOR AUTONOMOUS TO BEGIN:
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        if (autoCase == 'A') { // follow paths for autonomous case A

        }
        else if (autoCase == 'B') { // follow paths for autonomous case B

        }
        else if (autoCase == 'C') { // follow paths for autonomous case C
            drive.followTrajectory(shootStartingRings);
        }
        else telemetry.addLine("Error: Check auto case determination in program."); // debugging message
    }

    void pause(long waitTime) { // pause function to prevent new commands from being executed
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + waitTime) {}
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

            final int RING_SECTION_CROP_Y1 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENT_Y1);
            final int RING_SECTION_CROP_Y2 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENT_Y2);
            final int RING_SECTION_CROP_X1 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENT_X1);
            final int RING_SECTION_CROP_X2 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENT_X2);
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

            int pixels = pipeline.getRingPixels();
            telemetry.addData("Auto Case: ", (char) (autoCase));
            telemetry.addData("Ring Percentage: ", ringImagePercent);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Ring-Colored Pixels: ", pixels);
            telemetry.addData("Ring crop region X1", RING_SCAN_CROP_PERCENT_X1);
            telemetry.addData("Ring crop region Y1", RING_SCAN_CROP_PERCENT_Y1);
            telemetry.addData("Ring crop region X2", RING_SCAN_CROP_PERCENT_X2);
            telemetry.addData("Ring crop region Y2", RING_SCAN_CROP_PERCENT_Y2);
            telemetry.addData("Total pixels", pipeline.totalPixels);
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
