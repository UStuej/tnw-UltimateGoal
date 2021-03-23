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

@Config
@Autonomous(group = "drive")
public class WobbleGoalDeliveryAutonomous extends LinearOpMode {
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

    private Mat image;
    private Mat imageHSV;
    private Mat ringMask;
    private int pixels;

    @Override
    public void runOpMode() throws InterruptedException {
        lowerBound = new Scalar(RING_COLOR_H_START, RING_COLOR_S_START, RING_COLOR_V_START);
        upperBound = new Scalar(RING_COLOR_H_END, RING_COLOR_S_END, RING_COLOR_V_END);

        telemetry.addLine("Initializing OpenCV VideoCapture on index 0");

        // Initialize OpenCV VideoCapture
        videoFeed = new VideoCapture(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(initialPose);

        // Initialize the wobble goal pickup
        wgPickup = hardwareMap.get(Servo.class, "WGPickup");

        // Assuming the builder function units are in inches

        Trajectory moveOut = drive.trajectoryBuilder(initialPose)
                .strafeRight(6)
                .build();

        // Case A

        Trajectory deliver1A = drive.trajectoryBuilder(moveOut.end())
                .splineTo(new Vector2d(12, -58), Math.toRadians(180))
                .build();

        Trajectory collect2A = drive.trajectoryBuilder(deliver1A.end())
                .splineTo(new Vector2d(-48, 30), Math.toRadians(270))
                .build();

        Trajectory deliver2A = drive.trajectoryBuilder(collect2A.end())
                .splineTo(new Vector2d(0, -58), Math.toRadians(180))
                .build();

        Trajectory parkA = drive.trajectoryBuilder(deliver2A.end())
                .splineTo(new Vector2d(8, -32), Math.toRadians(0))
                .build();

        // Case B

        Trajectory deliver1B = drive.trajectoryBuilder(moveOut.end())
                .splineTo(new Vector2d(36, -36), Math.toRadians(225))
                .build();

        Trajectory collect2B = drive.trajectoryBuilder(deliver1B.end())
                .splineTo(new Vector2d(-48, -30), Math.toRadians(270))
                .build();

        Trajectory deliver2B = drive.trajectoryBuilder(collect2B.end())
                .splineTo(new Vector2d(28, -44), Math.toRadians(225))
                .build();

        Trajectory parkB = drive.trajectoryBuilder(deliver2B.end())
                .splineTo(new Vector2d(12, -60), Math.toRadians(0))
                .build();

        // Case C

        Trajectory deliver1C = drive.trajectoryBuilder(moveOut.end())
                .splineTo(new Vector2d(60, -48), Math.toRadians(180))
                .build();

        Trajectory collect2C = drive.trajectoryBuilder(deliver1C.end())
                .splineTo(new Vector2d(-48, 30), Math.toRadians(270))
                .build();

        Trajectory deliver2C = drive.trajectoryBuilder(collect2C.end())
                .splineTo(new Vector2d(-48, -58), Math.toRadians(180))
                .build();

        Trajectory parkC = drive.trajectoryBuilder(deliver2C.end())
                .splineTo(new Vector2d(-12, -58), Math.toRadians(180))
                .build();

        telemetry.addLine("Waiting for start...");

        waitForStart();

        if (isStopRequested()) return;

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
            currentCase = 1;  // Default to case A
        }

        if (currentCase == 1) {
            telemetry.addLine("Moving out");
            drive.followTrajectory(moveOut);

            telemetry.addLine("Moving to deliver 1A");
            drive.followTrajectory(deliver1A);

            telemetry.addLine("Lowering pickup");
            wgPickup.setPosition(0.7);

            telemetry.addLine("Moving to collect 2A");
            drive.followTrajectory(collect2A);

            telemetry.addLine("Raising pickup");
            wgPickup.setPosition(0.32);

            telemetry.addLine("Moving to deliver 2A");
            drive.followTrajectory(deliver2A);
            PoseStorage.wobbleGoal2RedPosition = new Pose2d(0, -58);  // TODO: Do likewise for all autonomous wobble goal movements. Also TODO: Headings should be included here

            telemetry.addLine("Lowering pickup");
            wgPickup.setPosition(0.7);

            telemetry.addLine("Moving to park A");
            drive.followTrajectory(parkA);
        }
        else if (currentCase == 2) {
            telemetry.addLine("Moving out");
            drive.followTrajectory(moveOut);

            telemetry.addLine("Moving to deliver 1B");
            drive.followTrajectory(deliver1B);

            telemetry.addLine("Lowering pickup");
            wgPickup.setPosition(0.7);

            telemetry.addLine("Moving to collect 2B");
            drive.followTrajectory(collect2B);

            telemetry.addLine("Raising pickup");
            wgPickup.setPosition(0.32);

            telemetry.addLine("Moving to deliver 2B");
            drive.followTrajectory(deliver2B);

            telemetry.addLine("Lowering pickup");
            wgPickup.setPosition(0.7);

            telemetry.addLine("Moving to park B");
            drive.followTrajectory(parkB);
        }
        else if (currentCase == 3) {
            telemetry.addLine("Moving out");
            drive.followTrajectory(moveOut);

            telemetry.addLine("Moving to deliver 1C");
            drive.followTrajectory(deliver1C);

            telemetry.addLine("Lowering pickup");
            wgPickup.setPosition(0.7);

            telemetry.addLine("Moving to collect 2C");
            drive.followTrajectory(collect2C);

            telemetry.addLine("Raising pickup");
            wgPickup.setPosition(0.32);

            telemetry.addLine("Moving to deliver 2C");
            drive.followTrajectory(deliver2C);

            telemetry.addLine("Lowering pickup");
            wgPickup.setPosition(0.7);

            telemetry.addLine("Moving to park C");
            drive.followTrajectory(parkC);
        }
        else {
            // Error here
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
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

    public int getCase() {
        int rings = getRings();

        telemetry.addData("Estimated ring count: ", rings);

        if (rings == 1) {
            return 2;
        }

        if (rings >= 4) {
            return 3;
        }

        return 1;  // Zero rings
    }
}
