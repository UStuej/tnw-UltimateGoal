package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * Main autonomous for iteration 1 of the robot. It scans a section of pixels in an image taken
 * by the phone camera for ring colors, counts those pixels, converts that to a number of rings,
 * then finally runs a preset trajectory based on the size of the ring stack, delivering the
 * wobble goals to the correct target
 */
@Config
@Autonomous(group = "drive")
public class WebcamRingDetection extends LinearOpMode {
    OpenCvCamera webcam;
    WebcamRingDetection.imageFeedPipeline pipeline = new WebcamRingDetection.imageFeedPipeline();

    public static double RING_SCAN_CROP_PERCENT_X1 = 0.1;  // 0.0
    public static double RING_SCAN_CROP_PERCENT_X2 = 0.9;  // 1.0
    public static double RING_SCAN_CROP_PERCENT_Y1 = 0.49;  // .49
    public static double RING_SCAN_CROP_PERCENT_Y2 = 0.75;  // .75

    double ringImagePercent = 0.0;

    public static final int RING_COLOR_H_START = 13;  // 15
    public static final int RING_COLOR_S_START = 45;  // 51
    public static final int RING_COLOR_V_START = 0;  // 0
    public static final int RING_COLOR_H_END = 35;  // 32
    public static final int RING_COLOR_S_END = 92;  // 89
    public static final int RING_COLOR_V_END = 100;  // 100

    char autoCase = 'X';

    final Pose2d initialPose = new Pose2d(-63, -48, Math.toRadians(90));

    private Servo wgPickup;
    private Servo ringDump;

    private Mat image;
    private Mat imageHSV;
    private Mat ringMask;
    private int pixels;

    private static double JOYSTICK_INPUT_THRESHOLD = 0.10;  // The global threshold for all joystick axis inputs under which no input will be registered. Also referred to as a deadzone

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
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });


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

    }

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
        gamepad1XHeld = gamepad1.x;
        gamepad1YHeld = gamepad1.y;
        gamepad1LeftShoulderHeld = gamepad1.left_bumper;
        gamepad1RightShoulderHeld = gamepad1.right_bumper;
        gamepad1LeftStickHeld = gamepad1.left_stick_button;
        gamepad1RightStickHeld = gamepad1.right_stick_button;
        gamepad1DpadUpHeld = gamepad1.dpad_up;
        gamepad1DpadDownHeld = gamepad1.dpad_down;
        gamepad1DpadLeftHeld = gamepad1.dpad_left;
        gamepad1DpadRightHeld = gamepad1.dpad_right;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;
        gamepad1XPressed = !gamepad1XWasHeld && gamepad1XHeld;
        gamepad1YPressed = !gamepad1YWasHeld && gamepad1YHeld;
        gamepad1LeftShoulderPressed = !gamepad1LeftShoulderWasHeld && gamepad1LeftShoulderHeld;
        gamepad1RightShoulderPressed = !gamepad1RightShoulderWasHeld && gamepad1RightShoulderHeld;
        gamepad1LeftStickPressed = !gamepad1LeftStickWasHeld && gamepad1LeftStickHeld;
        gamepad1RightStickPressed = !gamepad1RightStickWasHeld && gamepad1RightStickHeld;
        gamepad1DpadUpPressed = !gamepad1DpadUpWasHeld && gamepad1DpadUpHeld;
        gamepad1DpadDownPressed = !gamepad1DpadDownWasHeld && gamepad1DpadDownHeld;
        gamepad1DpadLeftPressed = !gamepad1DpadLeftWasHeld && gamepad1DpadLeftHeld;
        gamepad1DpadRightPressed = !gamepad1DpadRightWasHeld && gamepad1DpadRightHeld;

        // Cache previous gamepad 2 inputs
        boolean gamepad2AWasHeld = gamepad2AHeld;  // Whether or not the gamepad 2 a button was held
        boolean gamepad2BWasHeld = gamepad2BHeld;  // Whether or not the gamepad 2 b button was held
        boolean gamepad2XWasHeld = gamepad2XHeld;  // Whether or not the gamepad 2 x button was held
        boolean gamepad2YWasHeld = gamepad2YHeld;  // Whether or not the gamepad 2 y button was held
        boolean gamepad2LeftShoulderWasHeld = gamepad2LeftShoulderHeld;  // Whether or not the gamepad 2 left shoulder button was held
        boolean gamepad2RightShoulderWasHeld = gamepad2RightShoulderHeld;  // Whether or not the gamepad 2 left shoulder button was held
        boolean gamepad2LeftStickWasHeld = gamepad2LeftStickHeld;  // Whether or not the gamepad 2 left stick button was held
        boolean gamepad2RightStickWasHeld = gamepad2RightStickHeld;  // Whether or not the gamepad 2 right stick button was held
        boolean gamepad2DpadUpWasHeld = gamepad2DpadUpHeld;  // Whether or not the gamepad 2 dpad up button was held
        boolean gamepad2DpadDownWasHeld = gamepad2DpadDownHeld;  // Whether or not the gamepad 2 dpad down button was held
        boolean gamepad2DpadLeftWasHeld = gamepad2DpadLeftHeld;  // Whether or not the gamepad 2 dpad left button was held
        boolean gamepad2DpadRightWasHeld = gamepad2DpadRightHeld;  // Whether or not the gamepad 2 dpad right button was held

        // Get new values from the actual gamepad 2
        gamepad2AHeld = gamepad2.a;
        gamepad2BHeld = gamepad2.b;
        gamepad2XHeld = gamepad2.x;
        gamepad2YHeld = gamepad2.y;
        gamepad2LeftShoulderHeld = gamepad2.left_bumper;
        gamepad2RightShoulderHeld = gamepad2.right_bumper;
        gamepad2LeftStickHeld = gamepad2.left_stick_button;
        gamepad2RightStickHeld = gamepad2.right_stick_button;
        gamepad2DpadUpHeld = gamepad2.dpad_up;
        gamepad2DpadDownHeld = gamepad2.dpad_down;
        gamepad2DpadLeftHeld = gamepad2.dpad_left;
        gamepad2DpadRightHeld = gamepad2.dpad_right;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad2APressed = !gamepad2AWasHeld && gamepad2AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad2BPressed = !gamepad2BWasHeld && gamepad2BHeld;
        gamepad2XPressed = !gamepad2XWasHeld && gamepad2XHeld;
        gamepad2YPressed = !gamepad2YWasHeld && gamepad2YHeld;
        gamepad2LeftShoulderPressed = !gamepad2LeftShoulderWasHeld && gamepad2LeftShoulderHeld;
        gamepad2RightShoulderPressed = !gamepad2RightShoulderWasHeld && gamepad2RightShoulderHeld;
        gamepad2LeftStickPressed = !gamepad2LeftStickWasHeld && gamepad2LeftStickHeld;
        gamepad2RightStickPressed = !gamepad2RightStickWasHeld && gamepad2RightStickHeld;
        gamepad2DpadUpPressed = !gamepad2DpadUpWasHeld && gamepad2DpadUpHeld;
        gamepad2DpadDownPressed = !gamepad2DpadDownWasHeld && gamepad2DpadDownHeld;
        gamepad2DpadLeftPressed = !gamepad2DpadLeftWasHeld && gamepad2DpadLeftHeld;
        gamepad2DpadRightPressed = !gamepad2DpadRightWasHeld && gamepad2DpadRightHeld;

        // Gamepad 1 axes
        gamepad1LeftStickX = (Math.abs(gamepad1.left_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.left_stick_x : 0.0;  // Apply deadzone. Values below the deadzone are "snapped" to zero
        gamepad1LeftStickY = (Math.abs(gamepad1.left_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.left_stick_y : 0.0;
        gamepad1RightStickX = (Math.abs(gamepad1.right_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.right_stick_x : 0.0;
        gamepad1RightStickY = (Math.abs(gamepad1.right_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.right_stick_y : 0.0;
        gamepad1LeftTrigger = (Math.abs(gamepad1.left_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.left_trigger : 0.0;
        gamepad1RightTrigger = (Math.abs(gamepad1.right_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.right_trigger : 0.0;

        // Gamepad 2 axes
        gamepad2LeftStickX = (Math.abs(gamepad2.left_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.left_stick_x : 0.0;  // Apply deadzone. Values below the deadzone are "snapped" to zero
        gamepad2LeftStickY = (Math.abs(gamepad2.left_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.left_stick_y : 0.0;
        gamepad2RightStickX = (Math.abs(gamepad2.right_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.right_stick_x : 0.0;
        gamepad2RightStickY = (Math.abs(gamepad2.right_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.right_stick_y : 0.0;
        gamepad2LeftTrigger = (Math.abs(gamepad2.left_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.left_trigger : 0.0;
        gamepad2RightTrigger = (Math.abs(gamepad2.right_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.right_trigger : 0.0;
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
        final int resolutionTuner = 1; // One pixel sampled every # pixels.  Raise for speed, lower for reliability.
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
            Mat hsvInput = new Mat();
            Mat threshold = new Mat();
            final Scalar green = new Scalar(0.0d, 255.0d, 0.0d);
            final Scalar black = new Scalar (0.0d, 0.0d, 0.0d);
            final Scalar lowRingHSV = new Scalar(90, 0, 0);
            final Scalar hiRingHSV = new Scalar(255, 100, 100);

            setpixelPoint = new Point(0, 0);

            totalPixels = ((RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) * (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1) / resolutionTuner);
            int oneRingPixelsMinimum = (int) (totalPixels / resolutionTuner * oneRingPercentageMinimum); // Where maxXResolution is the number of pixels in a row
            int fourRingPixelsMinimum = (int) (totalPixels / resolutionTuner * fourRingPercentageMinimum); // Where maxXResolution is the number of pixels in a row
            Imgproc.cvtColor(imageFeed, hsvInput, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsvInput, lowRingHSV, hiRingHSV, threshold);

            ringPixels = 0;
            /*for(int x = RING_SECTION_CROP_X1; x <= RING_SECTION_CROP_X2; x += resolutionTuner) {
                for (int y = RING_SECTION_CROP_Y1; y <= RING_SECTION_CROP_Y2; y += resolutionTuner) {
                    pixel = imageFeed.get(y, x);
                    greenValue = pixel[1];
                    blueValue = pixel[2];
                    if (greenValue >= blueValue * orangeGBRatioLowThreshold) {
                        ringPixels++;
                        // imageFeed.set(y, x, setpixel)
                        setpixelPoint.x = (int) x;
                        setpixelPoint.y = (int) y;
                        Imgproc.circle(imageFeed, setpixelPoint, 1, ringVisualizeColor, Imgproc.FILLED);
                    }
                    else Imgproc.circle(imageFeed, setpixelPoint, 1, black, Imgproc.FILLED);
                }
            }*/

           /* Imgproc.rectangle(
                    imageFeed,
                    new Point(
                            RING_SECTION_CROP_X1,
                            RING_SECTION_CROP_Y1),
                    new Point(
                            RING_SECTION_CROP_X2,
                            RING_SECTION_CROP_Y2),
                    new Scalar(0, 255, 0), 4);*/


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

            return threshold;
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
