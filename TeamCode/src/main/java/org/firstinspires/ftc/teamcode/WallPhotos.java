package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class WallPhotos {

    /**Takes an image of a wall photo from the robot camera and returns either the estimated orientation of the robot given the difference in slopes between the sides of the contour enclosing the wall photo, or {@code null} if no such positive match was made
     */
    public static void getOrientationFromWallPhoto(final Mat img) {

        Mat edges = new Mat();

        {
            Mat resized = new Mat();
            Imgproc.resize(img, resized, new Size((300.0 * img.width()) / img.height(), 300.0));
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(resized, blurred, new Size(15.0, 15.0), 0.0);

            Mat hsv = new Mat();
            Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

            Mat thresholded = new Mat();
            Core.inRange(hsv, new Scalar(0.0, 0.0, 100.0), new Scalar(180.0, 60.0, 255.0), thresholded);  // FIXME This method doesn't have a thresholding type parameter, unlike its Python equivalent
            Imgproc.dilate(thresholded, thresholded, new Mat(), new Point(-1.0, -1.0), 1);

            Mat bf = new Mat();
            Imgproc.bilateralFilter(thresholded, bf, 11, 17.0, 17.0);
            Imgproc.Canny(bf, edges, 30.0, 200.0);

        }

    }

}
