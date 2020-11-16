package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class WallPhotos {

    public static void getOrientationFromWallPhoto(final Mat img) {

        {
            Mat resized = new Mat();
            Imgproc.resize(img, resized, new Size((300.0 * img.width()) / img.height(), 300.0));
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(resized, blurred, new Size(15.0, 15.0), 0.0);

            Mat hsv = new Mat();
            Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);
        }

    }

}
