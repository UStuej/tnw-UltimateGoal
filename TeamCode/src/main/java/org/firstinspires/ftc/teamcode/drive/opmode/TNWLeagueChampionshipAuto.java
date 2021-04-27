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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(group = "drive")
public class TNWLeagueChampionshipAuto extends LinearOpMode {
    OpenCvCamera webcam;
    TNWLeagueChampionshipAuto.imageFeedPipeline pipeline = new TNWLeagueChampionshipAuto.imageFeedPipeline();

// STARTER STACK DETECTION VALUES:
    public static double RING_SCAN_CROP_PERCENT_X1 = 0.4;  // 0.0
    public static double RING_SCAN_CROP_PERCENT_X2 = 0.8;  // 1.0
    public static double RING_SCAN_CROP_PERCENT_Y1 = 0.1;  // .49
    public static double RING_SCAN_CROP_PERCENT_Y2 = 0.6;  // .75

    double ringImagePercent = 0.0;
    double oneRingPercentageMinimum = .005; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 1 ring scenario
    double fourRingPercentageMinimum = .10; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 4 ring scenario

    private static final int RING_COLOR_H_START = 13;  // 15
    private static final int RING_COLOR_S_START = 45;  // 51
    private static final int RING_COLOR_V_START = 0;  // 0
    private static final int RING_COLOR_H_END = 35;  // 32
    private static final int RING_COLOR_S_END = 92;  // 89
    private static final int RING_COLOR_V_END = 100;  // 100

    char autoCase = 'X';


// ROADRUNNER VALUES:
    //Constant Roadrunner Pose Values
    final Pose2d initialPose = new Pose2d(-63, -32, Math.toRadians(0));
    final Pose2d powerShotShoot1 = new Pose2d(-4, -4, Math.toRadians(356));
    final int distanceBetweenPowerShots = 8; // inches
    final Pose2d targetZoneA1 = new Pose2d(22, -50, Math.toRadians(90));


// MOTOR AND SERVO DECLARATION:
    // Intake motor
    private DcMotor intakeDrive;

    // Wobble Goal manipulation motors and servos
    private Servo wobbleClaw;  // Wobble goal claw servo
    private Servo fingerServo;  // Ring finger servo
    private DcMotor wobbleArm;  // Wobble goal arm motor (used with encoders and runToPosition, so it acts like a servo)

    // Ring shooter motor
    private DcMotorEx ringShooter;

    // Ring Elevator motor
    private DcMotor ringElevator;

    // MOTOR AND SERVO POSITION CONSTANTS:
    private static int highGoalTPS = 57 * 28;  // Ticks per second of the shooter when active and aiming for the high goal
    private static int powerShotTPS = 50 * 28; // Ticks per second of the shooter when active and aiming for the power shots

    private static double CLAW_OPENED_POSITION = 0.24;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.80;  // The position of the claw when it is closed

    private static int ARM_DOWN_POSITION_DELTA = 402;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's down
    private static int ARM_UP_POSITION_DELTA = 221;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up
    private static int ARM_HOVER_POSITION_DELTA = 300;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up

    private static int ARM_DOWN_POSITION;  // The absolute position (in motor encoder units) of the arm's down position. Set on init
    private static int ARM_UP_POSITION;  // The absolute position (in motor encoder units) of the arm's up position. Set on init
    private static int ARM_HOVER_POSITION;  // The absolute position (in motor encoder units) of the arm's hover position. Set on init

    private static int RING_ELEVATOR_UP_POSITION;  // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION;  // The position of the Ring Elevator when it is in the DOWN state
    private static double RING_ELEVATOR_POWER = 0.7;  // The power for the motor to use when running to its target position

    private static double RING_FINGER_IN_POSITION = 0.23;  // The position of the ring finger when it's in
    private static double RING_FINGER_OUT_POSITION = 0.75;  // The position of the ring finger when it's out


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

        drive.setPoseEstimate(initialPose);

        telemetry.addData("Status: ", "Building Trajectories...");

// ROADRUNNER AUTONOMOUS TRAJECTORIES:

        // Case A
        Trajectory toPowerShotsA = drive.trajectoryBuilder(initialPose) // Drives to first power shots while initialising necessary motors
                .lineToLinearHeading(powerShotShoot1)
                .addSpatialMarker(new Vector2d(-30, -30), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // set ring finger to out position
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring bucket
                        ringShooter.setVelocity(powerShotTPS);
                    }
                })
                .addDisplacementMarker(new MarkerCallback() {  // When destination reached, shoot first power shot
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot first power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootSecondPowerShotA = drive.trajectoryBuilder(toPowerShotsA.end()) // Shoots first power shot
                .lineToConstantHeading(new Vector2d(powerShotShoot1.getX(), powerShotShoot1.getY() - distanceBetweenPowerShots))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot second power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootLastPowerShotA = drive.trajectoryBuilder(shootSecondPowerShotA.end())
                .lineToConstantHeading(new Vector2d(powerShotShoot1.getX(), powerShotShoot1.getY() - distanceBetweenPowerShots * 2))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot last power shot
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // begin rotating wobble arm ahead of time to save time
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow ring bucket to be dropped
                    }
                })
                .build();

        Trajectory deliverWobbleGoal1A = drive.trajectoryBuilder(shootLastPowerShotA.end()) // Drives to Target Zone A and delivers first wobble goal
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // begin dropping ring elevator
                    }
                })
                .lineToLinearHeading(targetZoneA1)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION); // release and score first wobble goal
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // retract ring finger to prevent accidental damage
                        pause(500);
                    }
                })
                .build();

        Trajectory backFromWobbleGoal1A = drive.trajectoryBuilder(deliverWobbleGoal1A.end()) // Backs up from the first wobble goal to allow resetting of the arm
                /*.addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_HOVER_POSITION); // raise wobble goal arm
                    }
                })*/
                .lineToLinearHeading(new Pose2d(targetZoneA1.getX(), targetZoneA1.getY() + 14, targetZoneA1.getHeading()))
                .build();

        Trajectory collectWobbleGoal2A = drive.trajectoryBuilder(backFromWobbleGoal1A.end()) // Drives to and collects second wobble goal for scoring
                .lineToLinearHeading(new Pose2d(-36, -52, Math.toRadians(0)))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);
                        pause(500);
                        wobbleArm.setTargetPosition(ARM_HOVER_POSITION);
                    }
                })
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

        if (autoCase == 'A') {
            drive.followTrajectory(toPowerShotsA);
            drive.followTrajectory(shootSecondPowerShotA);
            drive.followTrajectory(shootLastPowerShotA);
            drive.followTrajectory(deliverWobbleGoal1A);
            drive.followTrajectory(backFromWobbleGoal1A);
            drive.followTrajectory(collectWobbleGoal2A);
        }
        else if (autoCase == 'B') {

        }
        else if (autoCase == 'C') {

        }
        else telemetry.addLine("Error: Check auto case determination in program.");
    }

    int pixelCountToRings(int numPixels) {
        // Do something here to count the pixels. This might require both tweaking and estimation
        return numPixels/1000;  // Literally just guessed this value. Please don't rely on it
    }

    void pause(long waitTime) {
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
