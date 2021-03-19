package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;


@Config
@Autonomous(group = "drive")
public class TestRRAutonomous extends LinearOpMode {

    final Pose2d initialPose = new Pose2d(-63, -52, Math.toRadians(90));
    byte wobbleGoalTargetZone = -1; // 0, 1, and 2 correspond to target zones A, B, and C, respectively.

    private Servo wgPickup;

    //to do:
    // invert y-axis for other states

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(initialPose);

        // CASE A TRAJECTORIES:
        Trajectory strafeOut = drive.trajectoryBuilder(initialPose)
                .splineTo(new Vector2d(-57,-52), Math.toRadians(90))  // Strafe right 6
                .splineToLinearHeading(new Pose2d(-40, -58, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory deliver1A = drive.trajectoryBuilder(strafeOut.end(), true)
                .lineToConstantHeading(new Vector2d(12, -58))  // Back 52
                .build();

        Trajectory return1A = drive.trajectoryBuilder(deliver1A.end())
                .lineToLinearHeading(new Pose2d(-56, -58, Math.toRadians(180)))
                .build();

        Trajectory collect2A = drive.trajectoryBuilder(return1A.end())
                .lineToLinearHeading(new Pose2d(-48,-30, Math.toRadians(270)))
                .build();

        Trajectory toDeliverPositionA = drive.trajectoryBuilder(collect2A.end())
                .lineToLinearHeading(new Pose2d(-60,-58, Math.toRadians(180)))
                .build();

        Trajectory deliver2A = drive.trajectoryBuilder(toDeliverPositionA.end())
                .lineToLinearHeading(new Pose2d(0, -58, Math.toRadians(180)))
                .build();

        Trajectory parkA = drive.trajectoryBuilder(deliver2A.end())
                .splineTo(new Vector2d(-8, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-8,-40), Math.toRadians(135))
                .splineTo(new Vector2d(8,-32), Math.toRadians(0))
                .build();

        // CASE B TRAJECTORIES
        Trajectory deliver1B = drive.trajectoryBuilder(initialPose)
                .splineTo(new Vector2d(-57, -52), Math.toRadians(90))
                .splineTo(new Vector2d(-40, -58), Math.toRadians(180))
                .splineTo(new Vector2d(8, -58), Math.toRadians(180))
                .splineTo(new Vector2d(36,-36), Math.toRadians(225))
                .splineTo(new Vector2d(36, -36), Math.toRadians(225))
                .build();

        Trajectory return1B = drive.trajectoryBuilder(deliver1B.end())
                .splineTo(new Vector2d(8,-58), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -58), Math.toRadians(180))
                .build();

        Trajectory collect2B = drive.trajectoryBuilder(return1B.end())
                .splineTo(new Vector2d(-48,-30), Math.toRadians(270))
                .build();

        Trajectory toDeliverPositionB = drive.trajectoryBuilder(collect2B.end())
                .splineTo(new Vector2d(-60, -58), Math.toRadians(180))
                .build();

        Trajectory deliver2B = drive.trajectoryBuilder(toDeliverPositionB.end())
                .splineTo(new Vector2d(8, -58), Math.toRadians(180))  // Back 85
                .splineTo(new Vector2d(28,-44), Math.toRadians(225))
                .build();

        Trajectory parkB = drive.trajectoryBuilder(deliver2B.end())
                .splineTo(new Vector2d(16,56), Math.toRadians(225))  // Forward 12
                .splineTo(new Vector2d(12,-60), Math.toRadians(0))
                .build();

        // CASE C TRAJECTORIES
        Trajectory deliver1C = drive.trajectoryBuilder(initialPose)
                .splineTo(new Vector2d(-57, -52), Math.toRadians(90))  // Strafe right 6
                .splineTo(new Vector2d(-40,-58), Math.toRadians(180))
                .splineTo(new Vector2d(60, -58), Math.toRadians(180))  // Back 100
                .build();

        Trajectory return1C = drive.trajectoryBuilder(deliver1C.end())
                .splineTo(new Vector2d(-56, -58), Math.toRadians(180))  // Forward 116
                .build();

        Trajectory collect2C = drive.trajectoryBuilder(return1C.end())
                .splineTo(new Vector2d(-48,-30), Math.toRadians(270))
                .build();

        Trajectory toDeliverPositionC = drive.trajectoryBuilder(collect2C.end())
                .splineTo(new Vector2d(-60,-58), Math.toRadians(180))
                .build();

        Trajectory deliver2C = drive.trajectoryBuilder(toDeliverPositionC.end())
                .splineTo(new Vector2d(48, -58), Math.toRadians(180))  // Back 108
                .build();

        Trajectory parkC = drive.trajectoryBuilder(deliver2C.end())
                .splineTo(new Vector2d(-12, -58), Math.toRadians(180))  // Forward 60
                .build();

        wgPickup = hardwareMap.get(Servo.class, "WGPickup");

        if (gamepad1.a) {  // A is case A
            wobbleGoalTargetZone = 0;
        }
        else if (gamepad1.b) {  // B is case B
            wobbleGoalTargetZone = 1;
        }
        else if (gamepad1.x) {  // X is case C
            wobbleGoalTargetZone = 2;
        }
        else {
            wobbleGoalTargetZone = 0;  // Default to case A
        }

        waitForStart();

        if (wobbleGoalTargetZone == 0) {
            telemetry.addLine("Using target zone A");
            telemetry.addLine("Following delivery 1A");
            telemetry.update();
            drive.followTrajectory(strafeOut);
            drive.followTrajectory(deliver1A);
            telemetry.addLine("Lowering pickup");
            telemetry.update();
            wgPickup.setPosition(.7);  // 0.7 is the down position
            telemetry.addLine("Following return 1A");
            telemetry.update();
            drive.followTrajectory(return1A);
            telemetry.addLine("Following collect 2A");
            telemetry.update();
            drive.followTrajectory(collect2A);
            telemetry.addLine("Raising pickup");
            telemetry.update();
            wgPickup.setPosition(.32);  // 0.32 is the up position
            telemetry.addLine("Following deliver position A");
            telemetry.update();
            drive.followTrajectory(toDeliverPositionA);
            telemetry.addLine("Following deliver 2A");
            telemetry.update();
            drive.followTrajectory(deliver2A);
            telemetry.addLine("Lowering pickup");
            telemetry.update();
            wgPickup.setPosition(.7);
            telemetry.addLine("Following park A");
            telemetry.update();
            drive.followTrajectory(parkA);
        }
        else if (wobbleGoalTargetZone == 1) {
            telemetry.addLine("Using target zone B");
            telemetry.addLine("Following deliver 1B");
            telemetry.update();
            drive.followTrajectory(deliver1B);
            telemetry.addLine("Lowering pickup");
            telemetry.update();
            wgPickup.setPosition(.7);
            telemetry.addLine("Following return 1B");
            telemetry.update();
            drive.followTrajectory(return1B);
            telemetry.addLine("Following collect 2B");
            telemetry.update();
            drive.followTrajectory(collect2B);
            telemetry.addLine("Raising pickukp");
            telemetry.update();
            wgPickup.setPosition(.32);
            telemetry.addLine("Following deliver position B");
            telemetry.update();
            drive.followTrajectory(toDeliverPositionB);
            telemetry.addLine("Following deliver position 2B");
            telemetry.update();
            drive.followTrajectory(deliver2B);
            telemetry.addLine("Lowering pickup");
            telemetry.update();
            wgPickup.setPosition(.7);
            telemetry.addLine("Following park B");
            telemetry.update();
            drive.followTrajectory(parkB);
        }
        else if (wobbleGoalTargetZone == 2) {
            telemetry.addLine("Using target zone C");
            telemetry.update();
            telemetry.update();
            telemetry.addLine("Following deliver 1C");
            telemetry.update();
            drive.followTrajectory(deliver1C);
            telemetry.addLine("Lowering pickup");
            telemetry.update();
            wgPickup.setPosition(.7);
            telemetry.addLine("Following return 1C");
            telemetry.update();
            drive.followTrajectory(return1C);
            telemetry.addLine("Following collect 2C");
            telemetry.update();
            drive.followTrajectory(collect2C);
            telemetry.addLine("Raising pickup");
            telemetry.update();
            wgPickup.setPosition(.32);
            telemetry.addLine("Following deliver position C");
            telemetry.update();
            drive.followTrajectory(toDeliverPositionC);
            telemetry.addLine("Following deliver position 2C");
            telemetry.update();
            drive.followTrajectory(deliver2C);
            telemetry.addLine("Lowering pickup");
            telemetry.update();
            wgPickup.setPosition(.7);
            telemetry.addLine("Following park C");
            telemetry.update();
            drive.followTrajectory(parkC);
        }
        else {
            // Invalid target zone. Probably error here or something telemetry-wise
        }
        //if (isStopRequested())  // Probably not needed

    }
}
