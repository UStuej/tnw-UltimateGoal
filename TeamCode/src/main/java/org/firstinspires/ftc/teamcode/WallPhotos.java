package org.firstinspires.ftc.teamcode;

import org.jetbrains.annotations.NotNull;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;

public class WallPhotos {

    private static final double AVG_WALL_IMAGE_AREA = 100.0;
    private static final double WALL_IMAGE_AREA_THRESH = 10.0;

    /** Takes an image of a wall photo from the robot camera and returns either the estimated orientation of the robot given the difference in slopes between the sides of the contour enclosing the wall photo, or {@code null} if no such positive match was made.
     */
    public static void getOrientationFromWallPhoto(final @NotNull Mat img) {

        Mat imgMod = img.clone();
        Mat edges = new Mat();

        {
            Mat resized = new Mat();
            Imgproc.resize(img, resized, new Size((300.0 * img.width()) / img.height(), 300.0));
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(resized, blurred, new Size(15.0, 15.0), 0.0);

            Mat hsv = new Mat();
            Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

            Mat thresholded = new Mat();
            Core.inRange(hsv, new Scalar(0.0, 0.0, 100.0), new Scalar(180.0, 60.0, 255.0), thresholded);
            Imgproc.dilate(thresholded, thresholded, new Mat(), new Point(-1.0, -1.0), 1);

            Mat bf = new Mat();
            Imgproc.bilateralFilter(thresholded, bf, 11, 17.0, 17.0);
            Imgproc.Canny(bf, edges, 30.0, 200.0);

        }

        List<MatOfPoint> contourHulls = new ArrayList<>();

        {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Collections.sort(contours, new Comparator<Mat>() {
                @Override
                public int compare(Mat c0, Mat c1) {
                    return (int) Math.round(Imgproc.contourArea(c1) - Imgproc.contourArea(c0));
                }
            });
            contours = contours.subList(0, 10);

            for (MatOfPoint contour : contours) {
                MatOfInt contourHullIdx = new MatOfInt();
                Imgproc.convexHull(contour, contourHullIdx, true);
                MatOfPoint contourHull = pointsFromIdx(contour, contourHullIdx);
                if (Imgproc.contourArea(contourHull) > 9000.0)
                    contourHulls.add(contourHull);
            }
            
        }

        List<MatOfPoint2f> candidates1 = new ArrayList<>();

        {
            List<MatOfPoint2f> candidates0 = new ArrayList<>();

            for (MatOfPoint contour : contourHulls) {
                MatOfPoint2f contour2f = new MatOfPoint2f();
                contour.convertTo(contour2f, CvType.CV_32F);
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approxCurve, Imgproc.arcLength(contour2f, true) * 0.015, true);
                if (approxCurve.total() == 4L)
                    candidates0.add(approxCurve);
            }

            {
                List<MatOfPoint> candidates0Int = new ArrayList<>(candidates0.size());
                for (int i = 0; i < candidates0.size(); i++) {
                    MatOfPoint candidate = new MatOfPoint();
                    candidates0.get(i).convertTo(candidate, CvType.CV_32S);
                    candidates0Int.add(candidate);
                }
                Imgproc.drawContours(imgMod, candidates0Int, -1, new Scalar(0.0, 0.0, 0.0), 3);
            }

            for (MatOfPoint2f contour : candidates0) {
                if (Math.abs(Imgproc.contourArea(contour) - AVG_WALL_IMAGE_AREA) > WALL_IMAGE_AREA_THRESH)
                    candidates1.add(contour);
            }

            if (candidates1.size() == 0)
                // FIXME Send a warning to telemetry with the message "No 4-point contours were detected." (or some other action)
                return;

        }

        // Rectangle-ish tie break
        Collections.sort(candidates1, new Comparator<MatOfPoint2f>() {
            @Override
            public int compare(MatOfPoint2f c0, MatOfPoint2f c1) {
                return (int) (operation(c1) - operation(c0));
            }
            public long operation(MatOfPoint2f c) {
                Mat boxPoints = new Mat();
                Imgproc.boxPoints(Imgproc.minAreaRect(c), boxPoints);
                return Math.round(Math.abs(Imgproc.contourArea(c) / Imgproc.contourArea(boxPoints)));
            }
        });
        for (MatOfPoint2f contour : candidates1) {
            Point[] points = new Point[4];
            Imgproc.minAreaRect(contour).points(points);
            Imgproc.rectangle(imgMod, points[0], points[2], new Scalar(0.0, 255.0, 0.0));
        }

        // Area tie break
        Collections.sort(candidates1, new Comparator<MatOfPoint2f>() {
            @Override
            public int compare(MatOfPoint2f c0, MatOfPoint2f c1) {
                return 0;  // FIXME Return a value based on the difference found on line 119 in the original code.py
            }
        });

        MatOfPoint2f contour = candidates1.get(0);

    }

    private static MatOfPoint pointsFromIdx(@NotNull MatOfPoint src, @NotNull MatOfInt idx) {

        Point[] srcArr = src.toArray();
        int[] idxArr = idx.toArray();
        Point[] dstArr = new Point[idxArr.length];

        for (int i = 0; i < idxArr.length; i++) {
            dstArr[i] = srcArr[idxArr[i]];
        }

        return new MatOfPoint(dstArr);

    }

}
