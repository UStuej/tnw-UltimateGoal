package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestRRAutonomous extends LinearOpMode {

    final Pose2d initialPose = new Pose2d(-63, -52, Math.toRadians(90));
    byte wobbleGoalTargetZone = -1; // 0, 1, and 2 correspond to target zones A, B, and C, respectively.

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(initialPose);


        // UNIVERSAL TRAJECTORIES:




        // CASE A TRAJECTORIES:
        Trajectory deliver1A = drive.trajectoryBuilder(initialPose)
                .strafeRight(6)
                .splineTo(new Vector2d(-40, -58), Math.toRadians(180))
                .back(52)
                .build();

        Trajectory return1A = drive.trajectoryBuilder(deliver1A.end())
                .forward(68)
                .build();

        Trajectory collect2A = drive.trajectoryBuilder(return1A.end())
                .splineTo(new Vector2d(-48,-30), Math.toRadians(270))
                .build();

        Trajectory toDeliverPositionA = drive.trajectoryBuilder(collect2A.end())
                .splineTo(new Vector2d(-60,-58), Math.toRadians(180))
                .build();

        Trajectory deliver2A = drive.trajectoryBuilder(toDeliverPositionA.end())
                .back(60)
                .build();

        Trajectory parkA = drive.trajectoryBuilder(deliver2A.end())
                .forward(8)
                .splineTo(new Vector2d(-8,-40), Math.toRadians(135))
                .splineTo(new Vector2d(8,-32), Math.toRadians(0))
                .build();

        // CASE B TRAJECTORIES
        Trajectory deliver1B = drive.trajectoryBuilder(initialPose)
                .strafeRight(6)
                .splineTo(new Vector2d(-40, -58), Math.toRadians(180))
                .back(48)
                .splineTo(new Vector2d(36,-36), Math.toRadians(225))
                .build();

        Trajectory return1B = drive.trajectoryBuilder(deliver1B.end())
                .splineTo(new Vector2d(8,-58), Math.toRadians(180))
                .forward(64)
                .build();

        Trajectory collect2B = drive.trajectoryBuilder(return1B.end())
                .splineTo(new Vector2d(-48,-30), Math.toRadians(270))
                .build();

        Trajectory toDeliverPositionB = drive.trajectoryBuilder(collect2B.end())
                .splineTo(new Vector2d(-60, -58), Math.toRadians(180))
                .build();

        Trajectory deliver2B = drive.trajectoryBuilder(toDeliverPositionB.end())
                .back(68)
                .splineTo(new Vector2d(28,-44), Math.toRadians(225))
                .build();

        Trajectory parkB = drive.trajectoryBuilder(deliver2B.end())
                .forward(12)
                .splineTo(new Vector2d(12,-60), Math.toRadians(0))
                .build();

        // CASE C TRAJECTORIES
        Trajectory deliver1C = drive.trajectoryBuilder(initialPose)
                .strafeRight(6)
                .splineTo(new Vector2d(-40,-58), Math.toRadians(180))
                .back(100)
                .build();

        Trajectory return1C = drive.trajectoryBuilder(deliver1C.end())
                .forward(116)
                .build();

        Trajectory collect2C = drive.trajectoryBuilder(return1C.end())
                .splineTo(new Vector2d(-48,-30), Math.toRadians(270))
                .build();

        Trajectory toDeliverPositionC = drive.trajectoryBuilder(collect2C.end())
                .splineTo(new Vector2d(-60,-58), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;


        }
    }
