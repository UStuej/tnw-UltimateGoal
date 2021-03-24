package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import java.util.Vector;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestRRAutonomous extends LinearOpMode {
    //  The section of the image where the ring colors are checked
    private static final int RING_SECTION_CROP_Y1 = 0;  // TODO: Assign these
    private static final int RING_SECTION_CROP_Y2 = 0;
    private static final int RING_SECTION_CROP_X1 = 0;
    private static final int RING_SECTION_CROP_X2 = 0;

    // The range of colors that can be detected as rings
    private static final int RING_COLOR_H_START = 0;
    private static final int RING_COLOR_S_START = 0;
    private static final int RING_COLOR_V_START = 20;

    private static final int RING_COLOR_H_END = 255;
    private static final int RING_COLOR_S_END = 255;
    private static final int RING_COLOR_V_END = 255;

    final Pose2d initialPose = new Pose2d(-63, -52, Math.toRadians(90));

    public static VideoCapture videoFeed;

    public static Scalar lowerBound;
    public static Scalar upperBound;

    private Servo wgPickup;
    private Servo ringDump;

    private Mat image;
    private Mat imageHSV;
    private Mat ringMask;
    private int pixels;

    @Override
    public void runOpMode() throws InterruptedException {
        lowerBound = new Scalar(RING_COLOR_H_START, RING_COLOR_S_START, RING_COLOR_V_START);
        upperBound = new Scalar(RING_COLOR_H_END, RING_COLOR_S_END, RING_COLOR_V_END);

        // Initialize OpenCV VideoCapture
        videoFeed = new VideoCapture(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(initialPose);

        // Initialize the wobble goal pickup
        wgPickup = hardwareMap.get(Servo.class, "WGPickup");
        ringDump = hardwareMap.get(Servo.class, "ringDump");

        // Case A

        Trajectory deliver1A = drive.trajectoryBuilder(initialPose)             // Distance from wall, then drive to target zone A, while facing the tower goal
                .splineToConstantHeading(new Vector2d(-51, -52), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-36, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(27, -60), Math.toRadians(0))
                .build();

        Trajectory toLowGoalA = drive.trajectoryBuilder(deliver1A.end())         // Drives forward to completely release wobble goal, then drives to base of tower goal facing drop zone
                .splineToConstantHeading(new Vector2d(38, -60), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(64, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory driveToCollect2A = drive.trajectoryBuilder(toLowGoalA.end())
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(27)))
                .build();

        Trajectory collect2A = drive.trajectoryBuilder(driveToCollect2A.end())
                .lineTo(new Vector2d(-48, -24))
                .build();

        Trajectory deliver2A = drive.trajectoryBuilder(collect2A.end(), true)
                .splineTo(new Vector2d(-24, -60), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(3, -60, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory parkA = drive.trajectoryBuilder(deliver2A.end())
                .splineToConstantHeading(new Vector2d(-8, -60), Math.toRadians(0))
                .splineTo(new Vector2d(-16, -52), Math.toRadians(0))
                .splineTo(new Vector2d(-8, -44), Math.toRadians(0))
                .splineTo(new Vector2d(8, -32), Math.toRadians(0))
                .build();

        // Case B

        Trajectory deliver1B = drive.trajectoryBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-51, -52), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, -40, Math.toRadians(60)), Math.toRadians(0))
                .build();

        Trajectory toLowGoalB = drive.trajectoryBuilder(deliver1B.end())
                .splineToConstantHeading(new Vector2d(56, -20), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d( 56, -20.1, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(64, -36), Math.toRadians(0))
                .build();

        Trajectory collect2B = drive.trajectoryBuilder(toLowGoalB.end())
                .splineToConstantHeading(new Vector2d(56, -36), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(27)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48, -24), Math.toRadians(0))
                .build();

        Trajectory deliver2B = drive.trajectoryBuilder(collect2B.end(), true)
                .splineTo(new Vector2d(-24, -60), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(24, -32, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory parkB = drive.trajectoryBuilder(deliver2B.end())
                .splineToConstantHeading(new Vector2d(12, -32), Math.toRadians(0))
                .build();

        // Case C

        /*Trajectory deliver1C = drive.trajectoryBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-51, -52), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();

        Trajectory toLowGoalC = drive.trajectoryBuilder(deliver1C.end())
                .splineToConstantHeading(new Vector2d(48, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(64, -36), Math.toRadians(0))
                .build();

        Trajectory collect2C = drive.trajectoryBuilder(toLowGoalC.end())
                .splineToConstantHeading(new Vector2d(56, -36), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(27)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48, -24), Math.toRadians(0))
                .build();

        Trajectory deliver2C = drive.trajectoryBuilder(collect2C.end(), true)
                .splineTo(new Vector2d(-24, -60), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, -60, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory parkC = drive.trajectoryBuilder(deliver2C.end())
                .splineToConstantHeading(new Vector2d(12, -60), Math.toRadians(0))
                .build();*/


        waitForStart();

        // if (isStopRequested()) return;

        //int currentCase = getCase();

        int currentCase;  // TODO: Finish OpenCV code that figures this out

        if (gamepad1.a) {  // A is case A
            currentCase = 1;
        }
        else if (gamepad1.b) {  // B is case B
            currentCase = 2;
        }
        else if (gamepad1.x) {  // X is case C
            currentCase = 3;
        }
        else {
            currentCase = 2;  // Default to case A
        }

        if (currentCase == 1) {
            drive.followTrajectory(deliver1A);
            wgPickup.setPosition(.70); // Drop wobble goal pickup
            pause(750);
            drive.followTrajectory(toLowGoalA);
            ringDump.setPosition(.83); // Dump preloaded rings
            pause(200);
            ringDump.setPosition(.48);
            pause(50);
            ringDump.setPosition(.83);
            pause(500);
            ringDump.setPosition(.48);
            wgPickup.setPosition(.70);
            drive.followTrajectory(driveToCollect2A);
            drive.followTrajectory(collect2A);
            wgPickup.setPosition(.32);
            pause(500);
            drive.followTrajectory(deliver2A);
            wgPickup.setPosition(.70);
            pause(750);
            drive.followTrajectory(parkA);
        }
        else if (currentCase == 2) {
            drive.followTrajectory(deliver1B);
            wgPickup.setPosition(.70); // Drop wobble goal pickup
            pause(750);
            drive.followTrajectory(toLowGoalB);
            ringDump.setPosition(.83); // Dump preloaded rings
            pause(200);
            ringDump.setPosition(.48);
            pause(50);
            ringDump.setPosition(.83);
            pause(500);
            ringDump.setPosition(.48);
            wgPickup.setPosition(.70);
            drive.followTrajectory(collect2B);
            wgPickup.setPosition(.32);
            pause(500);
            drive.followTrajectory(deliver2B);
            wgPickup.setPosition(.70);
            pause(500);
            drive.followTrajectory(parkB);
        }
        else if (currentCase == 3) {
   /*         drive.followTrajectory(deliver1C);
            wgPickup.setPosition(.70); // Drop wobble goal pickup
            pause(500);
            drive.followTrajectory(toLowGoalC);
            ringDump.setPosition(.83); // Dump preloaded rings
            pause(200);
            ringDump.setPosition(.48);
            pause(50);
            ringDump.setPosition(.83);
            pause(500);
            ringDump.setPosition(.48);
            wgPickup.setPosition(.70);
            drive.followTrajectory(collect2C);
            wgPickup.setPosition(.32);
            pause(500);
            drive.followTrajectory(deliver2C);
            wgPickup.setPosition(.70);
            pause(500);
            drive.followTrajectory(parkC);*/
        }
        else {
            // Error here
        }
    }

    public int getRings() {
        image = new Mat();
        ringMask = new Mat();
        videoFeed.read(image);

        // Crop the image to the search region
        image = image.submat(RING_SECTION_CROP_Y1, RING_SECTION_CROP_Y2, RING_SECTION_CROP_X1, RING_SECTION_CROP_X2);

        // Convert the image to the HSV colorspace for easier color detection
        Imgproc.cvtColor(image, imageHSV, Imgproc.COLOR_RGB2HSV);  // Assuming this implementation of OpenCV's VideoCapture object reads pixels in the RGB colorspace rather than the BGR one

        // Threshold the HSV image based on our ring search colors
        Core.inRange(imageHSV, lowerBound, upperBound, ringMask);

        // Count the number of nonzero pixels in the mask
        pixels = Core.countNonZero(ringMask);

        return pixelCountToRings(pixels);
    }

    int pixelCountToRings(int numPixels) {
        // Do something here to count the pixels. This might require both tweaking and estimation
        return numPixels/1000;  // Literally just guessed this value. Please don't rely on it
    }

    void pause(long waitTime) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + waitTime) {}
    }

    public int getCase() {
        int rings = getRings();

        if (rings == 1) {
            return 2;
        }

        if (rings >= 4) {
            return 3;
        }

        return 1;  // Zero rings
    }
}
