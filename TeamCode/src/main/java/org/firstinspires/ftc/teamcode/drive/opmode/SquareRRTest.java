package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(group = "drive")
public class SquareRRTest extends LinearOpMode {

    final Pose2d initialPose = new Pose2d(-63, -52, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(initialPose);

        Trajectory strafeOut = drive.trajectoryBuilder(initialPose)
                .strafeRight(12)
                .build();

        Trajectory step1 = drive.trajectoryBuilder(strafeOut.end())
                .splineTo(new Vector2d(0,-10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24, -30), Math.toRadians(0))
                .build();

        Trajectory right1 = drive.trajectoryBuilder(step1.end())
                .strafeRight(24)
                .build();




        waitForStart();

        drive.followTrajectory(strafeOut);
        drive.followTrajectory(step1);
        drive.followTrajectory(right1);

        }
    }
