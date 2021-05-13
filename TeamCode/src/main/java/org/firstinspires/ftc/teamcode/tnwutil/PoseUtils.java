package org.firstinspires.ftc.teamcode.tnwutil;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseUtils {
    public Pose2d forwardInches (double inches, Pose2d currentPose) {
        return new Pose2d(currentPose.getX() + currentPose.headingVec().getX(),
                currentPose.getY() + currentPose.headingVec().getY(), currentPose.getHeading());
    }
}
