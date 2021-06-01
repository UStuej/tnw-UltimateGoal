package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.tnwutil.PoseStorage;

@Disabled
@Config
@Autonomous(group = "drive")
public class AutoTeleOp extends LinearOpMode {
    private static double CLAW_MINIMUM = 0.0;  // The minimum position for the wobble goal shoulder servo
    private static double CLAW_MAXIMUM = 1.0;  // The maximum position for the wobble goal shoulder servo

    private static double PICKUP_MINIMUM = 0.0;  // The minimum position for the wobble goal shoulder servo
    private static double PICKUP_MAXIMUM = 1.0;  // The maximum position for the wobble goal shoulder servo

    private static double SHOULDER_OUT_POSITION = 0.24;  // The position of the shoulder when it is out
    private static double SHOULDER_IN_POSITION = 0.68;  // The position of the shoulder when it is in

    private static double CLAW_OPENED_POSITION = 0.66;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.13;  // The position of the claw when it is closed

    private static double PICKUP_UP_POSITION = 0.32;  // The position of the pickup when it is up
    private static double PICKUP_DOWN_POSITION = 0.70;  // The position of the pickup when it is down

    private static double RING_DUMP_DUMP_POSITION = 0.83;  // The position of the ring dump when it's dumping
    private static double RING_DUMP_COLLECT_POSITION = 0.48;  // The position of the ring dump when it's collecting

    // Intake motor object
    private DcMotor intakeDrive;

    // Intake motor power
    private double intakeDrivePower;

    // Wobble Goal manipulation motors and servos
    private DcMotor wobbleLift;
    private Servo wobblePickup;
    private Servo wobbleShoulder;
    private Servo wobbleClaw;

    // Ring manipulation servo
    private Servo ringDump;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);
    }
}
