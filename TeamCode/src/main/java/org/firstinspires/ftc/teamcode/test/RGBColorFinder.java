package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Disabled
@Config
@Autonomous(group = "drive")
public class RGBColorFinder extends LinearOpMode {
    OpenCvCamera webcam;
    RGBColorFinder.imageFeedPipeline pipeline = new RGBColorFinder.imageFeedPipeline();

    final boolean MANUAL_CURRENT_CASE = false;
    final double JOYSTICK_DEADZONE = 0.1;

    double RING_SCAN_CROP_PERCENT_X1 = 0.2;  // 0.0
    double RING_SCAN_CROP_PERCENT_X2 = 0.5;  // 1.0
    double RING_SCAN_CROP_PERCENT_Y1 = 0.3;  // .49
    double RING_SCAN_CROP_PERCENT_Y2 = 1.0;  // .75

    double ringImagePercent = 0.0;

    private static final int RING_COLOR_H_START = 13;  // 15
    private static final int RING_COLOR_S_START = 45;  // 51
    private static final int RING_COLOR_V_START = 0;  // 0
    private static final int RING_COLOR_H_END = 35;  // 32
    private static final int RING_COLOR_S_END = 92;  // 89
    private static final int RING_COLOR_V_END = 100;  // 100

    char autoCase = 'X';

    //Constant Roadrunner Values
    final Pose2d initialPose = new Pose2d(-63, -24, Math.toRadians(0));
    final Pose2d powerShotShoot1 = new Pose2d(-4, -4, Math.toRadians(356));
    final int distanceBetweenPowerShots = 8; // inches


    private Servo wgPickup;
    private Servo ringDump;

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
                        //move ringFinger to out position
                        //move ringElevator to up position
                        //spin up flywheel
                    }
                })
                .build();
        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        Trajectory shootPowerShotsA = drive.trajectoryBuilder(toPowerShotsA.end()) // Shoots preloaded rings to score 3 power shots while keeping aim by strafing
                .lineToConstantHeading(new Vector2d(powerShotShoot1.getX(), powerShotShoot1.getY() - distanceBetweenPowerShots * 2))
                .addSpatialMarker(new Vector2d(powerShotShoot1.getX(), powerShotShoot1.getY()), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        //shoot first ring
                    }
                })
                .addSpatialMarker(new Vector2d (powerShotShoot1.getX(), powerShotShoot1.getY() - distanceBetweenPowerShots), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        //shoot second Ring
                    }
                })
                .addSpatialMarker(new Vector2d(powerShotShoot1.getX(), powerShotShoot1.getY() - distanceBetweenPowerShots * 2), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        //shoot last ring
                    }
                })
                .build();

        Trajectory deliverWobbleGoal1 = drive.trajectoryBuilder(shootPowerShotsA.end()) // Drives to Target Zone A and delivers first wobble goal
                .lineToLinearHeading(new Pose2d(8, -56, Math.toRadians(320)))
                .addSpatialMarker(new Vector2d(8, -52), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        //drop wg1
                    }
                })
                .build();

        // Servo Init Positions:

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        /*
         * Send some stats to the telemetry
         */
        if (ringImagePercent >= .002 && ringImagePercent < .01) { autoCase = 'B'; }
        else if (ringImagePercent >= .01) { autoCase = 'C'; }
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

        //if (isStopRequested()) return;

        //int currentCase = getCase();

        int currentCase = 0;

        if (MANUAL_CURRENT_CASE) {
            if (gamepad1.a) {  // A is case A
                currentCase = 1;
            } else if (gamepad1.b) {  // B is case B
                currentCase = 2;
            } else if (gamepad1.x) {  // X is case C
                currentCase = 3;
            } else {
                currentCase = 2;  // Default to case A
            }
        }
        else {
            if (autoCase == 'A') {
                currentCase = 1;
            }
            else if (autoCase == 'B') {
                currentCase = 2;
            }
            else if (autoCase == 'C') {
                currentCase = 3;
            }
        }

        if (currentCase != 1 && currentCase != 2 && currentCase != 3) {
            telemetry.addLine("WARNING: No valid case detected");
            telemetry.addLine("Pausing one second before proceeding");
            telemetry.update();

            pause(1000);
        }

        if (currentCase == 1) {

        }
        else if (currentCase == 2) {

        }
        else if (currentCase == 3) {

        }
        else {
            // Error here
        }
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
        final double orangeGBRatioLowThreshold = 1.5;
        double oneRingPercentageMinimum = .001; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 1 ring scenario
        double fourRingPercentageMinimum = .004; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 4 ring scenario
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
            boolean orangePixel = false;

            setpixelPoint = new Point(0, 0);

            totalPixels = ((RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) * (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1) / resolutionTuner);
            int oneRingPixelsMinimum = (int) (totalPixels / resolutionTuner * oneRingPercentageMinimum); // Where maxXResolution is the number of pixels in a row
            int fourRingPixelsMinimum = (int) (totalPixels / resolutionTuner * fourRingPercentageMinimum); // Where maxXResolution is the number of pixels in a row

            ringPixels = 0;
            /*for(int x = RING_SECTION_CROP_X1; x <= RING_SECTION_CROP_X2; x += resolutionTuner) {
                for (int y = RING_SECTION_CROP_Y1; y <= RING_SECTION_CROP_Y2; y += resolutionTuner) {
                    pixel = imageFeed.get(y, x);
                    redValue = pixel[0];
                    greenValue = pixel[1];
                    blueValue = pixel[2];
                    if (redValue >= RED_VALUE_MIN && greenValue >= blueValue * orangeGBRatioLowThreshold) {
                        ringPixels++;
                        // imageFeed.set(y, x, setpixel)
                        setpixelPoint.x = (int) x;
                        setpixelPoint.y = (int) y;
                        Imgproc.circle(imageFeed, setpixelPoint, 2, ringVisualizeColor, Imgproc.FILLED);
                    }
                }
            }*/

            setpixelPoint.y = Math.abs(RING_SECTION_CROP_Y1 + (RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) / 2);
            setpixelPoint.x = Math.abs(RING_SECTION_CROP_X1 + (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1) / 2);
            pixel = imageFeed.get((int) setpixelPoint.y, (int) setpixelPoint.x);

            redValue = pixel[0];
            greenValue = pixel[1];
            blueValue = pixel[2];

            if(redValue >= blueValue * 5 && redValue >= greenValue * 2 && greenValue >= blueValue * 2) orangePixel = true;

            Imgproc.circle(imageFeed, setpixelPoint, 1, ringVisualizeColor, Imgproc.FILLED);

            redValue = pixel[0]; greenValue = pixel[1]; blueValue = pixel[2];



            Imgproc.rectangle(
                    imageFeed,
                    new Point(
                            RING_SECTION_CROP_X1,
                            RING_SECTION_CROP_Y1),
                    new Point(
                            RING_SECTION_CROP_X2,
                            RING_SECTION_CROP_Y2),
                    new Scalar(0, 255, 0), 4);


            ringImagePercent = (double) ringPixels / (double) totalPixels;

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

             /*
              * Send some stats to the telemetry
              */

            char autoCase_ = 'X';

            if (ringImagePercent >= .002 && ringImagePercent < .01) { autoCase_ = 'B'; }
            else if (ringImagePercent >= .01) { autoCase_ = 'C'; }
            else {autoCase_ = 'A'; }

            int pixels = pipeline.getRingPixels();
            telemetry.addData("Orange? ", orangePixel);
            telemetry.addData("R: ", redValue);
            telemetry.addData("G: ", greenValue);
            telemetry.addData("B: ", blueValue);
            telemetry.addData("Auto Case: ", (char) (autoCase_));
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
