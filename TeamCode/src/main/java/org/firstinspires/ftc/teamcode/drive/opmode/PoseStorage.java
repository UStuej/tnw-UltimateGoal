package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseStorage {
    // The current pose of the robot
    public static Pose2d currentPose = new Pose2d(-63, -48, Math.toRadians(90));

    // The poses of the wobble goals
    public static Pose2d wobbleGoal1RedPosition = new Pose2d(-63, -48, Math.toRadians(90));  // These may not be valid until TestRRAutonomous is completed
    public static Pose2d wobbleGoal2RedPosition = new Pose2d(-24, -34, Math.toRadians(90));
    public static Pose2d wobbleGoal1BluePosition = new Pose2d(-48, -24);
    public static Pose2d wobbleGoal2BluePosition = new Pose2d(-48, -48);
    public static Pose2d ringStack1Position = new Pose2d(-24, 36);
    public static Pose2d ringStack2Position = new Pose2d(-24, -36);

    public static Pose2d powerShot1Pose = new Pose2d(0, -4, Math.toRadians(270));  // Assuming the robot starts moving forward (TODO: Test these ASAP)
    public static Pose2d powerShot2Pose = new Pose2d(0, -12, Math.toRadians(270));
    public static Pose2d powerShot3Pose = new Pose2d(0, -20, Math.toRadians(270));
    public static Pose2d highGoalShootPose = new Pose2d();  // TODO
}