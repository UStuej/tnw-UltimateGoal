package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.tnwutil.collections.Pair;
import org.firstinspires.ftc.teamcode.tnwutil.collections.Triplet;
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
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class WallPhotos {

    private static final double AVG_WALL_IMAGE_AREA = 100.0;  // The average area, in pixels, an image on the field wall will take up. Currently set to a dummy value.
    private static final double WALL_IMAGE_AREA_THRESH = 10.0;  // The area, in pixels, a wall image may differentiate from the average wall image area while still being considered as a wall image. Also set to a dummy value for now.

    /**
     * Takes an image of a wall photo from the robot camera and returns either the estimated orientation of the robot given the difference in slopes between the sides of the contour enclosing the wall photo, or {@code null} if no such positive match was made.
     *
     * @param img The input image.  Should be of type {@code CV_8UC3}.
     */
    public static void getOrientationFromWallPhoto(final @NotNull Mat img) {

        Mat imgMod = img.clone();
        Mat edges = new Mat();

        // Prepare the input image for processing.
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

        List<MatOfPoint> contourHulls = new ArrayList<>();  // The convex hulls of the found contours.

        {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Collections.sort(contours, new Comparator<Mat>() {
                @Override
                public int compare(Mat c0, Mat c1) {
                    return (int) Math.round(Imgproc.contourArea(c1) - Imgproc.contourArea(c0));
                }
            });
            contours = contours.subList(0, 10); // FIXME Maybe add this integer value as an argument?

            for (MatOfPoint contour : contours) {
                MatOfInt contourHullIdx = new MatOfInt();
                Imgproc.convexHull(contour, contourHullIdx);  // Get the convex hulls of the contours.
                MatOfPoint contourHull = pointsFromIdx(contour, contourHullIdx);
                if (Imgproc.contourArea(contourHull) > 9000.0)  // Filter out very small contours.
                    contourHulls.add(contourHull);
            }
            
        }

        List<MatOfPoint2f> candidates1 = new ArrayList<>();  // The double-filtered candidate contours.

        {
            List<MatOfPoint2f> candidates0 = new ArrayList<>();  // The single-filtered candidate contours.

            for (MatOfPoint contour : contourHulls) {
                MatOfPoint2f contour2f = new MatOfPoint2f();
                contour.convertTo(contour2f, CvType.CV_32F);
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approxCurve, Imgproc.arcLength(contour2f, true) * 0.015, true);  // FIXME Also this 0.015.
                if (approxCurve.total() == 4L)  // Our wall photo will have 4 points, as it is a rectangle.
                    candidates0.add(approxCurve);  // Add it to the list of contours which might be our wall image.
            }

            {
                List<MatOfPoint> candidates0Int = new ArrayList<>(candidates0.size());
                for (MatOfPoint2f contour : candidates0) {
                    MatOfPoint contourInt = new MatOfPoint();
                    contour.convertTo(contourInt, CvType.CV_32S);
                    candidates0Int.add(contourInt);
                }
                Imgproc.drawContours(imgMod, candidates0Int, -1, new Scalar(0.0, 0.0, 0.0), 3);  // Draw the single-filtered candidates onto imgMod, a clone of img used for debugging and visual representation purposes.
            }

            for (MatOfPoint2f contour : candidates0) {
                if (Math.abs(Imgproc.contourArea(contour) - AVG_WALL_IMAGE_AREA) > WALL_IMAGE_AREA_THRESH)  // Our rectangle should also encompass a certain specified range of areas.
                    candidates1.add(contour);
            }

            // TODO (?) More criteria could be added here, such as a position test to make sure the contour isn't in a place it couldn't be under normal circumstances (such as on the top of the image).

            if (candidates1.size() == 0)
                // FIXME Send a warning to telemetry with the message "No 4-point contours were detected." (or some other action).
                return;

        }

        // FIXME The tie break priority (in the original Python code) represents more of the singular method used to determine which contour is chosen rather than a priority since there's currently no way to tell whether the tie remained after the new sorting method was applied.

        // Rectangle-ish tie break.
        // Break ties based on parallel sides.
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

        // Area tie break.
        // Break ties based on the calculated area difference.
//        Collections.sort(candidates1, new Comparator<MatOfPoint2f>() {
//            @Override
//            public int compare(MatOfPoint2f c0, MatOfPoint2f c1) {
//                return 0;  // FIXME Return a value based on the difference found on line 119 in the original code.py.
//            }
//        });

        for (MatOfPoint2f contour : candidates1) {
            Point[] points = new Point[4];
            Imgproc.minAreaRect(contour).points(points);
            Imgproc.rectangle(imgMod, points[0], points[2], new Scalar(0.0, 255.0, 0.0));
        }

        Point[] decidedContourArr = candidates1.get(0).toArray();  // Choose the contour with the highest score from the previous tie breaking algorithms.

        {
            List<MatOfPoint> candidates1Int = new ArrayList<>(candidates1.size());
            for (MatOfPoint2f contour : candidates1) {
                MatOfPoint contourInt = new MatOfPoint();
                contour.convertTo(contourInt, CvType.CV_32S);
                candidates1Int.add(contourInt);
            }
            Imgproc.drawContours(imgMod, candidates1Int, -1, new Scalar(255.0, 0.0, 0.0), 3);  // Draw the double-filtered contours from earlier onto imgMod.
        }
        {
            List<MatOfPoint> contourIntList = new ArrayList<>(1);
            MatOfPoint contourInt = new MatOfPoint();
            candidates1.get(0).convertTo(contourInt, CvType.CV_32S);
            contourIntList.add(contourInt);
            Imgproc.drawContours(imgMod, contourIntList, -1, new Scalar(0.0, 255.0, 0.0), 3);  // Draw the single winning contour decided by the tie-breaking algorithms.
        }

        // TODO Line 147 of source code.py (logging)

        List<Triplet<Pair<Point, Point>, Double, Double>> segSlpLens = new ArrayList<>();  // The segments themselves, their slopes, and their lengths for all possible line segments which can connect the points in the winning contour.

        {
            Set<Pair<Point, Point>> segments = new HashSet<>(decidedContourArr.length);  // The segments mentioned above.

            // Collect groupings of points from the winning contour as segments.
            for (Point point0 : decidedContourArr) {
                for (Point point1 : decidedContourArr) {
                    if (!(point0.x == point1.x && point0.y == point1.y))
                        segments.add(
                                (point0.x != point1.x)
                                ? ((point0.x > point1.x)
                                        ? new Pair<>(point1, point0)
                                        : new Pair<>(point0, point1))
                                : ((point0.y > point1.y)
                                        ? new Pair<>(point1, point0)
                                        : new Pair<>(point0, point1))
                        );  // This operation also removes duplicate line segments.
                }
            }

            // Group each segment with its corresponding slope and length.
            for (Pair<Point, Point> segment : segments) {
                if (segment.first != segment.second) {
                    segSlpLens.add(new Triplet<>(
                            segment,
                            (segment.second.y - segment.first.y) / (segment.second.x - segment.first.x),
                            Math.sqrt(Math.pow(segment.second.x - segment.first.x, 2.0) + Math.pow(segment.second.y - segment.first.y, 2.0)))
                    );
                }
            }
        }

        Collections.sort(segSlpLens, new Comparator<Triplet<Pair<Point, Point>, Double, Double>>() {
            @Override
            public int compare(Triplet<Pair<Point, Point>, Double, Double> ssl0, Triplet<Pair<Point, Point>, Double, Double> ssl1) {
                return (int) Math.round(ssl0.third - ssl1.third);
            }
        });  // Sort the segments by largest length.
        segSlpLens.remove(segSlpLens.size() - 1);  // Remove the longest two segments, which are likely the diagonals.
        segSlpLens.remove(segSlpLens.size() - 1);

        if (segSlpLens.size() != 4) {  // Ensure that there are only four remaining line segments, meaning that they should form a quadrilateral.
            // FIXME Send a warning to telemetry with an error message (or some other action).
            return;
        }

        int[][] sidesIdx = new int[][] {  // One of the two possible combinations of segment groupings.
                {0, 2},
                {1, 3}
        };

        // Draw the quadrilateral mentioned above.
        for (int[] sideIdx : sidesIdx) {
            Pair<Point, Point> line0 = segSlpLens.get(sideIdx[0]).first;
            Pair<Point, Point> line1 = segSlpLens.get(sideIdx[1]).first;

            Imgproc.line(imgMod, line0.first, line0.second, new Scalar(0.0, 0.0, 255.0), 5);
            Imgproc.line(imgMod, line1.first, line1.second, new Scalar(255.0, 0.0, 0.0), 5);
        }

        // TODO Write (likely Imgcodecs.imwrite) imgMod to a file (or output it somehow).
        // TODO Log/Output sides.length.

//        double lengthRatio;
//        for (int[] sideIdx : sidesIdx) {                                                            // FIXME Not really sure what this is for, but I transcribed it anyway.
//            lengthRatio = segSlpLens.get(sideIdx[0]).third / segSlpLens.get(sideIdx[1]).third;
//        }

        // TODO Somehow take the difference or quotient of the opposite sides' slopes to derive a rotation, and maybe combine this with the position of the contour relative to the image center to determine a relatively exact location.

    }

    /**
     * Convenience function to retrieve {@link Point}s from a {@link MatOfPoint} specified by an index array represented as a {@link MatOfInt}.
     *
     * @param src The {@code Point} array from which to retrieve values.
     * @param idx The {@code int} array specifying which indicies to retrieve values from in the {@code Point} array.
     * @return The
     *
     * @throws ArrayIndexOutOfBoundsException If an index in {@code idx} is greater than {@code (src.length - 1)}.
     */
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
