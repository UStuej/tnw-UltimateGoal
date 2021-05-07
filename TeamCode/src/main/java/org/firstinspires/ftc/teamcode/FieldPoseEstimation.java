package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.List;

public class FieldPoseEstimation {

    private static final int FIELD_WIDTH = 24*6;  // Inches
    private static final int FIELD_HEIGHT = 24*6;

    // Guessed at all these values. Thresholding might not even be necessary
    private static final Scalar HSV_RED_THRESHOLD_MIN = new Scalar(340.0, 175.0, 50.0);
    private static final Scalar HSV_RED_THRESHOLD_MAX = new Scalar(360.0, 255.0, 255.0);

    private static final int MAX_CONTOURS = 10;

    private static final int CONTOUR_AREA_THRESHOLD_MIN = 1000;

    private static final int MARKER_WIDTH = 2;  // Width of the red tape inward from the corners in inches

    private static final VideoCapture feed = new VideoCapture(0);  // FIXME This object will likely be initialized in the main TeleOp OpMode class, so use the one from there

    // The 4 outer points of the markers in field coordinates, with the ground at z zero
    private static final Point3[] marker1Points = {
            new Point3(24.0 * -2, 24.0 * 0, 0.0),  // Top left
            new Point3(24.0 * -2, 24.0 * 1, 0.0),  // Top right
            new Point3(24.0 * -3, 24.0 * 0, 0.0),  // Bottom left
            new Point3(24.0 * -3, 24.0 * 1, 0.0)   // Bottom right
    };

    private static final Point3[] marker2Points = {
            new Point3(24.0 * -1, 24.0 * 1, 0),  // Top left
            new Point3(24.0 * -1, 24.0 * 2, 0),  // Top right
            new Point3(24.0 * -2, 24.0 * 1, 0),  // Bottom left
            new Point3(24.0 * -2, 24.0 * 2, 0)   // Bottom right
    };

    private static final Point3[] marker3Points = {
            new Point3(24.0 * -2, 24.0 * 2, 0),  // Top left
            new Point3(24.0 * -2, 24.0 * 3, 0),  // Top right
            new Point3(24.0 * -3, 24.0 * 2, 0),  // Bottom left
            new Point3(24.0 * -3, 24.0 * 3, 0)   // Bottom right
    };


    /**
     * Given a list of images of a flat sheet of circles width*height large taken by an external camera and a copy of the circle grid printout, computes and returns camera intrinsics from calibration.
     */
    public static Pose2d circlesCalibrateCamera(final Mat[] images, final Mat printoutImage, int gridWidth, int gridHeight, int z, boolean asymmetricGrid) {

        Mat grayscalePrintout = new Mat(printoutImage.size(), printoutImage.type());
        Imgproc.cvtColor(printoutImage, grayscalePrintout, Imgproc.COLOR_BGR2GRAY);  // Convert the printout to grayscale

        Mat objectPoints = new Mat();  // FIXME There is no Javadoc for this method, so this is likely to be the cause of an issue.
        boolean success = Calib3d.findCirclesGrid(grayscalePrintout, new Size(gridWidth, gridHeight), objectPoints, asymmetricGrid ? Calib3d.CALIB_CB_ASYMMETRIC_GRID : Calib3d.CALIB_CB_SYMMETRIC_GRID);  // FIXME Boolean 'success' may not be necessary.

        List<Point3> objectPoints3d = new ArrayList<>((int) objectPoints.total());
        //for (Point point : objectPoints) {  // FIXME Convert MatOfPoint 'objectPoints' to an iterable object (likely a List).
        //    objectPoints3d.add(new Point3(point.x, point.y, z));
        //}

        return new Pose2d();  // FIXME Dummy
    }
    /**
     * Given a list of images of a flat sheet of circles width*height large taken by an external camera and a copy of the circle grid printout, computes and returns camera intrinsics from calibration.
     */
    public static Pose2d circlesCalibrateCamera(Mat[] images, Mat printoutImage, int gridWidth, int gridHeight, int z) {
        return circlesCalibrateCamera(images, printoutImage, gridWidth, gridHeight, z, true);
    }
    /**
     * Given a list of images of a flat sheet of circles width*height large taken by an external camera and a copy of the circle grid printout, computes and returns camera intrinsics from calibration.
     */
    public static Pose2d circlesCalibrateCamera(Mat[] images, Mat printoutImage, int gridWidth, int gridHeight) {
        return circlesCalibrateCamera(images, printoutImage, gridWidth, gridHeight, 0);
    }
    /**
     * Given a list of images of a flat sheet of circles width*height large taken by an external camera and a copy of the circle grid printout, computes and returns camera intrinsics from calibration.
     */
    public static Pose2d circlesCalibrateCamera(Mat[] images, Mat printoutImage) {
        return circlesCalibrateCamera(images, printoutImage, 4, 11);
    }

}
