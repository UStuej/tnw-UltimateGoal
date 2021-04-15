package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// TeleOp101: The Second

// This TeleOp, programmed for the second iteration of the robot,
// has the ability to feed rings into the shooting mechanism,
// adjust the power applied to the shooter motor via PID control,
// and automatically lift the bucket

@Config
@Autonomous(group = "drive")
public class TeleOp101 extends LinearOpMode {
    public static Pose2d currentPose = new Pose2d(-63, -52, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        // Init stuff
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(currentPose);

        waitForStart();

        if (isStopRequested()) return;

        // Do stuff
    }
}
