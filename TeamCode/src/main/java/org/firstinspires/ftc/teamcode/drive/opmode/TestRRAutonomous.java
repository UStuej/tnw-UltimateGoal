package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;
import org.opencv.imgproc.*;
import org.opencv.videoio.*;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Vector;

/*
 * Main autonomous for iteration 1 of the robot. It scans a section of pixels in an image taken
 * by the phone camera for ring colors, counts those pixels, converts that to a number of rings,
 * then finally runs a preset trajectory based on the size of the ring stack, delivering the
 * wobble goals to the correct target
 */
@Config
@Autonomous(group = "drive")
public class TestRRAutonomous extends LinearOpMode {
    OpenCvCamera phoneCam;
    TestRRAutonomous.imageFeedPipeline pipeline = new TestRRAutonomous.imageFeedPipeline();

    final boolean JOYSTICKS_ADJUST_BB = true;
    final boolean MANUAL_CURRENT_CASE = false;
    final double JOYSTICK_DEADZONE = 0.1;

    double RING_SCAN_CROP_PERCENT_X1 = 0.0;  // 0.0
    double RING_SCAN_CROP_PERCENT_X2 = 0.3;  // 1.0
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

    final Pose2d initialPose = new Pose2d(-63, -48, Math.toRadians(90));

    private Servo wgPickup;
    private Servo ringDump;

    private Mat image;
    private Mat imageHSV;
    private Mat ringMask;
    private int pixels;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(initialPose);

        // Initialize the wobble goal pickup
        wgPickup = hardwareMap.get(Servo.class, "WGPickup");
        ringDump = hardwareMap.get(Servo.class, "ringDump");

        telemetry.addData("Status: ", "Building Trajectories...");

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        // Case A

        Trajectory deliver1A1 = drive.trajectoryBuilder(initialPose)             // Distance from wall, then drive to target zone A, while facing the tower goal
                .lineTo(new Vector2d(-51, -48))
                .build();

        Trajectory deliver1A2 = drive.trajectoryBuilder(deliver1A1.end())
                .lineToLinearHeading(new Pose2d(27, -56, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(23, -56), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                    }
                })
                .build();


        Trajectory toLowGoalA1 = drive.trajectoryBuilder(deliver1A2.end())         // Drives forward to completely release wobble goal, then drives to base of tower goal facing drop zone
                .lineTo(new Vector2d(38, -56))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        Trajectory toLowGoalA2 = drive.trajectoryBuilder(toLowGoalA1.end())
                .lineToLinearHeading(new Pose2d(65, -32, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(63, -36), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringDump.setPosition(.83); // Dump preloaded rings
                    }
                })
                .build();

        Trajectory driveToCollect2A = drive.trajectoryBuilder(toLowGoalA2.end())
                .lineToLinearHeading(new Pose2d(-47, -7, Math.toRadians(90)))
                /*.splineToConstantHeading(new Vector2d(56, -52), Math.toRadians(0))*/
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                        ringDump.setPosition(.48); // Move ring dump to collect position
                    }
                })
                /*.splineToConstantHeading(new Vector2d(48, -56), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -56, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(270)), Math.toRadians(0))*/
                .build();

        Trajectory collect2A = drive.trajectoryBuilder(driveToCollect2A.end())
                .lineTo(new Vector2d(-48, -18))
                //.lineToConstantHeading(new Vector2d(-24, -30))
                .addSpatialMarker(new Vector2d(-24, -34), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        Trajectory deliver2A = drive.trajectoryBuilder(collect2A.end(), true)
                .splineTo(new Vector2d(-24, -56), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(4, -56, Math.toRadians(180)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(2, -56), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                    }
                })
                .build();

        Trajectory parkA = drive.trajectoryBuilder(deliver2A.end())
                .splineToConstantHeading(new Vector2d(-5, -56), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, -44), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(16, -32), Math.toRadians(0))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        // Case B

        Trajectory deliver1B1 = drive.trajectoryBuilder(initialPose)
                .lineTo(new Vector2d(-51, -48))
                .build();

        Trajectory deliver1B2 = drive.trajectoryBuilder(deliver1B1.end())
                .lineToLinearHeading(new Pose2d(-24, -54, Math.toRadians(60)))
                .build();

        Trajectory deliver1B3 = drive.trajectoryBuilder(deliver1B2.end())
                .lineTo(new Vector2d(46, -34))
                .addSpatialMarker(new Vector2d(42, -40), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                    }
                })
                .build();

        Trajectory toLowGoal1B = drive.trajectoryBuilder(deliver1B3.end())
                .lineTo(new Vector2d(51, -20))
                //.splineToSplineHeading(new Pose2d( 65, -36, Math.toRadians(180)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(65, -36), Math.toRadians(0))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        Trajectory toLowGoal2B = drive.trajectoryBuilder(toLowGoal1B.end())
                .lineToLinearHeading(new Pose2d(65, -32, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(63, -32), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringDump.setPosition(.83); // Dump preloaded rings
                    }
                })
                .build();

        Trajectory driveToCollect2B = drive.trajectoryBuilder(toLowGoal2B.end())
                .lineToLinearHeading(new Pose2d(-47, -6, Math.toRadians(90)))
                /*.splineToConstantHeading(new Vector2d(56, -52), Math.toRadians(0))*/
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                        ringDump.setPosition(.48); // Move ring dump to collect position
                    }
                })
                /*.splineToConstantHeading(new Vector2d(48, -56), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -56, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(270)), Math.toRadians(0))*/
                .build();

        Trajectory collect2B = drive.trajectoryBuilder(driveToCollect2B.end())
                .lineTo(new Vector2d(-47, -18))
                //.lineToConstantHeading(new Vector2d(-24, -30))
                .addSpatialMarker(new Vector2d(-24, -34), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        Trajectory deliver2B1 = drive.trajectoryBuilder(collect2B.end())
                .lineTo(new Vector2d(-47, -8))
                .build();

        Trajectory deliver2B2 = drive.trajectoryBuilder(deliver2B1.end(), true)
                .lineToLinearHeading(new Pose2d(24, -34, Math.toRadians(180)))
                /*.splineTo(new Vector2d(-24, -16), Math.toRadians(0))
                .splineTo(new Vector2d(12, -32), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(24, -32, Math.toRadians(180)), Math.toRadians(0))*/
                .addSpatialMarker(new Vector2d(18, -30), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                    }
                })
                .build();

        Trajectory parkB = drive.trajectoryBuilder(deliver2B2.end())
                .splineToConstantHeading(new Vector2d(12, -32), Math.toRadians(0))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        // Case C

        Trajectory deliver1C1 = drive.trajectoryBuilder(initialPose)
                .lineTo(new Vector2d(-51, -48))
                .build();

        Trajectory deliver1C2 = drive.trajectoryBuilder(deliver1C1.end())
                .lineToLinearHeading(new Pose2d(-24, -58, Math.toRadians(180)))
                .build();

        Trajectory deliver1C3 = drive.trajectoryBuilder(deliver1C2.end())
                .lineTo(new Vector2d(60, -58))
                .addSpatialMarker(new Vector2d(56, -58), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                    }
                })
                .build();


        Trajectory toLowGoal1C = drive.trajectoryBuilder(deliver1C3.end())
                .lineTo(new Vector2d(48, -58))
                //.splineToSplineHeading(new Pose2d( 65, -36, Math.toRadians(180)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(65, -36), Math.toRadians(0))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        Trajectory toLowGoal2C = drive.trajectoryBuilder(toLowGoal1C.end())
                .lineToLinearHeading(new Pose2d(65, -32, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(63, -32), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringDump.setPosition(.83); // Dump preloaded rings
                    }
                })
                .build();

        Trajectory driveToCollect2C = drive.trajectoryBuilder(toLowGoal2C.end())
                .lineToLinearHeading(new Pose2d(-47, -8, Math.toRadians(90)))
                /*.splineToConstantHeading(new Vector2d(56, -52), Math.toRadians(0))*/
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                        ringDump.setPosition(.48); // Move ring dump to collect position
                    }
                })
                /*.splineToConstantHeading(new Vector2d(48, -56), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -56, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(270)), Math.toRadians(0))*/
                .build();

        Trajectory collect2C = drive.trajectoryBuilder(driveToCollect2C.end())
                .lineTo(new Vector2d(-47, -18))
                //.lineToConstantHeading(new Vector2d(-24, -30))
                .addSpatialMarker(new Vector2d(-24, -34), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.32); // Raise wobble goal pickup
                    }
                })
                .build();

        Trajectory deliver2C1 = drive.trajectoryBuilder(collect2C.end())
                .lineTo(new Vector2d(-46, -52))
                .build();

        Trajectory deliver2C2 = drive.trajectoryBuilder(deliver2C1.end(), true)
                .lineToLinearHeading(new Pose2d(52, -55, Math.toRadians(180)))
                /*.splineTo(new Vector2d(-24, -16), Math.toRadians(0))
                .splineTo(new Vector2d(12, -32), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(24, -32, Math.toRadians(180)), Math.toRadians(0))*/
                .addSpatialMarker(new Vector2d(48, -55), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wgPickup.setPosition(.70); // Drop wobble goal pickup
                    }
                })
                .build();

        Trajectory parkC = drive.trajectoryBuilder(deliver2C2.end())
                .splineToConstantHeading(new Vector2d(12, -52), Math.toRadians(0))
                .build();

        // Servo Init Positions:
        wgPickup.setPosition(.32); // Raise wobble goal pickup
        ringDump.setPosition(.48); // Move ring dump to collect position

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
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
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
            drive.followTrajectory(deliver1A1);
            drive.followTrajectory(deliver1A2);  // Drops wobble goal at 23, -56
            PoseStorage.wobbleGoal1RedPosition = new Pose2d(23, -56, deliver1A2.end().getHeading());  // Store this position as the new wobble goal position
            drive.followTrajectory(toLowGoalA1);
            drive.followTrajectory(toLowGoalA2);
            drive.followTrajectory(driveToCollect2A);
            drive.followTrajectory(collect2A);  // Picks wobble goal up at -24, -34
            drive.followTrajectory(deliver2A);  // Drops wobble goal at 2, -56
            PoseStorage.wobbleGoal2RedPosition = new Pose2d(2, -56, deliver2A.end().getHeading());
            drive.followTrajectory(parkA);
            PoseStorage.currentPose = parkA.end();  // Set our ending pose for any TeleOp code that relies on it
        }
        else if (currentCase == 2) {
            drive.followTrajectory(deliver1B1);
            drive.followTrajectory(deliver1B2);
            drive.followTrajectory(deliver1B3);  // Drops wobble goal at 42, -40
            PoseStorage.wobbleGoal1RedPosition = new Pose2d(42, -40, deliver1B3.end().getHeading());
            drive.followTrajectory(toLowGoal1B);
            drive.followTrajectory(toLowGoal2B);
            pause(250);
            drive.followTrajectory(driveToCollect2B);
            drive.followTrajectory(collect2B);
            drive.followTrajectory(deliver2B1);
            drive.followTrajectory(deliver2B2);  // Drops wobble goal at 18, -30
            PoseStorage.wobbleGoal2RedPosition = new Pose2d(18, -30, deliver2B2.end().getHeading());
            drive.followTrajectory(parkB);
            PoseStorage.currentPose = parkB.end();
        }
        else if (currentCase == 3) {
            drive.followTrajectory(deliver1C1);
            drive.followTrajectory(deliver1C2);
            drive.followTrajectory(deliver1C3);  // Drops wobble goal at 56, -58
            PoseStorage.wobbleGoal1RedPosition = new Pose2d(56, -58, deliver1C3.end().getHeading());
            drive.followTrajectory(toLowGoal1C);
            drive.followTrajectory(toLowGoal2C);
            pause(250);
            drive.followTrajectory(driveToCollect2C);
            drive.followTrajectory(collect2C);
            drive.followTrajectory(deliver2C1);
            drive.followTrajectory(deliver2C2);  // Drops wobble goal at 48, -55
            PoseStorage.wobbleGoal2RedPosition = new Pose2d(48, -55, deliver2C2.end().getHeading());
            drive.followTrajectory(parkC);
            PoseStorage.currentPose = parkC.end();
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
        double greenValue, blueValue;
        final int resolutionTuner = 5; // One pixel sampled every # pixels.  Raise for speed, lower for reliability.
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

            setpixelPoint = new Point(0, 0);

            totalPixels = ((RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) * (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1) / resolutionTuner);
            int oneRingPixelsMinimum = (int) (totalPixels / resolutionTuner * oneRingPercentageMinimum); // Where maxXResolution is the number of pixels in a row
            int fourRingPixelsMinimum = (int) (totalPixels / resolutionTuner * fourRingPercentageMinimum); // Where maxXResolution is the number of pixels in a row

            ringPixels = 0;
            for(int x = RING_SECTION_CROP_X1; x <= RING_SECTION_CROP_X2; x += resolutionTuner) {
                for (int y = RING_SECTION_CROP_Y1; y <= RING_SECTION_CROP_Y2; y += resolutionTuner) {
                    pixel = imageFeed.get(y, x);
                    greenValue = pixel[1];
                    blueValue = pixel[2];
                    if (greenValue >= blueValue * orangeGBRatioLowThreshold) {
                        ringPixels++;
                        // imageFeed.set(y, x, setpixel)
                        setpixelPoint.x = (int) x;
                        setpixelPoint.y = (int) y;
                        //Imgproc.circle(imageFeed, setpixelPoint, 5, ringVisualizeColor, Imgproc.FILLED);
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
            telemetry.addData("Auto Case: ", (char) (autoCase_));
            telemetry.addData("Ring Percentage: ", ringImagePercent);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
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
                phoneCam.pauseViewport();
            }
            else
            {
                phoneCam.resumeViewport();
            }
        }
    }
}
