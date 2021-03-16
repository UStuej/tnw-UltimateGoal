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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
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

    public static VideoCapture videoFeed;

    public static Scalar lowerBound;
    public static Scalar upperBound;

    Mat image;
    Mat imageHSV;
    Mat ringMask;
    int pixels;

    @Override
    public void runOpMode() throws InterruptedException {
        lowerBound = new Scalar(RING_COLOR_H_START, RING_COLOR_S_START, RING_COLOR_V_START);
        upperBound = new Scalar(RING_COLOR_H_END, RING_COLOR_S_END, RING_COLOR_V_END);

        // Initialize OpenCV VideoCapture
        videoFeed = new VideoCapture(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Assuming the builder function units are in inches

        Trajectory case1Trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(6)
                .splineTo(new Vector2d(5*12, 9), Math.toRadians(90))
                .back(50)
                .build();

        Trajectory case2Trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(6)
                .splineTo(new Vector2d(5*12, 9), Math.toRadians(90))
                .back(65)
                .build();

        Trajectory case3Trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(6)
                .splineTo(new Vector2d(5*12, 9), Math.toRadians(90))
                .back(100)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        int currentCase = getCase();

        int currentCase = 0;  // TODO: Figure this out using OpenCV

        if (currentCase == 1) {
            drive.followTrajectory(case1Trajectory);
        }
        else if (currentCase == 2) {
            drive.followTrajectory(case2Trajectory);
        }
        else if (currentCase == 3) {
            drive.followTrajectory(case3Trajectory);
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
    }

    public int getCase() {
        int rings = getRings();

        if (rings == 0) {
            return 1;
        }

        if (rings == 1) {
            return 2;
        }

        if (rings >= 4) {
            return 3;
        }
    }
}
