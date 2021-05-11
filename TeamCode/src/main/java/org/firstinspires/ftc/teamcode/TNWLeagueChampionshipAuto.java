package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(group = "drive")
public class TNWLeagueChampionshipAuto extends LinearOpMode {
    OpenCvCamera webcam;
    TNWLeagueChampionshipAuto.imageFeedPipeline pipeline = new TNWLeagueChampionshipAuto.imageFeedPipeline();

// STARTER STACK DETECTION VALUES:
    public static double RING_SCAN_CROP_PERCENT_X1 = 0.4;  // 0.0
    public static double RING_SCAN_CROP_PERCENT_X2 = 0.8;  // 1.0
    public static double RING_SCAN_CROP_PERCENT_Y1 = 0.1;  // .49
    public static double RING_SCAN_CROP_PERCENT_Y2 = 0.6;  // .75

    double ringImagePercent = 0.0;
    double oneRingPercentageMinimum = .01; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 1 ring scenario
    double fourRingPercentageMinimum = .10; // A number between 0 and 1.  Tune to identify what percentage of pixels need to be orange for 4 ring scenario

    private static final int RING_COLOR_H_START = 13;  // 15
    private static final int RING_COLOR_S_START = 45;  // 51
    private static final int RING_COLOR_V_START = 0;  // 0
    private static final int RING_COLOR_H_END = 35;  // 32
    private static final int RING_COLOR_S_END = 92;  // 89
    private static final int RING_COLOR_V_END = 100;  // 100

    char autoCase = 'X';


// ROADRUNNER VALUES:
    // Constant Roadrunner Pose Values
    // Starting pose
    final Pose2d STARTING_POSE = new Pose2d(-63, -32, Math.toRadians(0));

    // Target zone poses
    final int DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING = 10; // inches
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_A = new Pose2d(-35, -52, Math.toRadians(15));
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_B = new Pose2d(-35, -52, Math.toRadians(15));
    final Pose2d SECOND_WOBBLE_GOAL_PICKUP_POSITION_C = new Pose2d(-35-4, -55-4, Math.toRadians(330));
    final Pose2d SECOND_WOBBLE_GOAL_ALIGN_POSITION = new Pose2d(SECOND_WOBBLE_GOAL_PICKUP_POSITION_B.getX() + 12, SECOND_WOBBLE_GOAL_PICKUP_POSITION_B.getY() + 12, SECOND_WOBBLE_GOAL_PICKUP_POSITION_B.getHeading());
    final Pose2d SECOND_WOBBLE_GOAL_ALIGN_POSITION_C = new Pose2d(SECOND_WOBBLE_GOAL_PICKUP_POSITION_C.getX() + 12, SECOND_WOBBLE_GOAL_PICKUP_POSITION_C.getY(), SECOND_WOBBLE_GOAL_PICKUP_POSITION_C.getHeading());
    final Pose2d TARGET_ZONE_A1 = new Pose2d(24, -51, Math.toRadians(90));
    final Pose2d TARGET_ZONE_A2 = new Pose2d(TARGET_ZONE_A1.getX(), TARGET_ZONE_A1.getY() + DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_A1.getHeading());
    final Pose2d TARGET_ZONE_B1 = new Pose2d(30, -34, Math.toRadians(180));
    final Pose2d TARGET_ZONE_B2 = new Pose2d(TARGET_ZONE_B1.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_B1.getY() + 2, TARGET_ZONE_B1.getHeading());
    final Pose2d TARGET_ZONE_C1 = new Pose2d(53, -50, Math.toRadians(180));
    final Pose2d TARGET_ZONE_C2 = new Pose2d(TARGET_ZONE_C1.getX() - DISTANCE_BETWEEN_WOBBLE_GOAL_SCORING, TARGET_ZONE_C1.getY(), TARGET_ZONE_C1.getHeading());
    // Starter stack related poses
    final Vector2d STARTER_STACK = new Vector2d(-24, -35);
    final Pose2d LONG_SHOT_POSE = new Pose2d(-37, -36, Math.toRadians(356));
    // Power shot related poses
    final Pose2d POWER_SHOT_SHOOT_1 = new Pose2d(-3, -3.5, Math.toRadians(356));
    final double DISTANCE_BETWEEN_POWER_SHOTS = 8; // inches
    // Parking pose
    final Pose2d PARKING_POSE = new Pose2d(12, -24, Math.toRadians(0));

// MOTOR AND SERVO DECLARATION:
    // Intake motor
    private DcMotor intakeDrive;

    // Wobble Goal manipulation motors and servos
    private Servo wobbleClaw;  // Wobble goal claw servo
    private Servo fingerServo;  // Finger servo
    private DcMotor wobbleArm;  // Wobble goal arm motor (used with encoders and runToPosition, so it acts like a servo)

    // Ring shooter motor
    private DcMotorEx ringShooter;

    // Ring Elevator motor
    private DcMotor ringElevator;

    // MOTOR AND SERVO POSITION CONSTANTS:
    private static double highGoalTPS = 57.5 * 28;  // Ticks per second of the shooter when active and aiming for the high goal
    private static double powerShotTPS = 50 * 28; // Ticks per second of the shooter when active and aiming for the power shots

    private static double CLAW_OPENED_POSITION = 0.24;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.80;  // The position of the claw when it is closed

    private static int ARM_DOWN_POSITION_DELTA = 415;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's down
    private static int ARM_UP_POSITION_DELTA = 221;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up
    private static int ARM_HOVER_POSITION_DELTA = 330;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up

    private static int ARM_DOWN_POSITION;  // The absolute position (in motor encoder units) of the arm's down position. Set on init
    private static int ARM_UP_POSITION;  // The absolute position (in motor encoder units) of the arm's up position. Set on init
    private static int ARM_HOVER_POSITION;  // The absolute position (in motor encoder units) of the arm's hover position. Set on init
    private static int ARM_STARTING_POSITION; // The absolute position (in motor encoder units) of the arm's starting position. Set on init

    private static int RING_ELEVATOR_UP_POSITION;  // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION;  // The position of the Ring Elevator when it is in the DOWN state
    private static double RING_ELEVATOR_POWER = 0.7;  // The power for the motor to use when running to its target position

    private static double RING_FINGER_IN_POSITION = 0.23;  // The position of the finger servo when it's in
    private static double RING_FINGER_OUT_POSITION = 0.75;  // The position of the finger servo when it's out

    private static double INTAKE_IN_POWER = -1;  // The power set to intake drive for collecting rings
    private static double INTAKE_OUT_POWER = INTAKE_IN_POWER * -1;  // The power set to intake drive for ejecting rings


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(STARTING_POSE);

        telemetry.addData("Status: ", "Building Trajectories...");

// ROADRUNNER AUTONOMOUS TRAJECTORIES:

// Case A Trajectories
        /*Trajectory toPowerShotsA = drive.trajectoryBuilder(STARTING_POSE) // Drives to and scores first power shots while initialising necessary motors
                .lineToLinearHeading(POWER_SHOT_SHOOT_1)
                .addSpatialMarker(new Vector2d(-40, -40), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // set finger servo to out position
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring bucket
                        ringShooter.setVelocity(powerShotTPS);
                    }
                })
                .addDisplacementMarker(new MarkerCallback() {  // When destination reached, shoot first power shot
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot first power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();*/
        Trajectory toPowerShotsA = drive.trajectoryBuilder(STARTING_POSE) // Drives to and scores first power shots while initialising necessary motors
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(0);
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // set finger servo to out position
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring bucket
                        ringShooter.setVelocity(powerShotTPS); // spin up flywheel to score power shots
                    }
                })
                .lineToLinearHeading(POWER_SHOT_SHOOT_1)
                .addDisplacementMarker(new MarkerCallback() {  // When destination reached, shoot first power shot
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot first power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootSecondPowerShotA = drive.trajectoryBuilder(toPowerShotsA.end()) // Shoots second power shot
                .lineToConstantHeading(new Vector2d(POWER_SHOT_SHOOT_1.getX(), POWER_SHOT_SHOOT_1.getY() - DISTANCE_BETWEEN_POWER_SHOTS))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot second power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootLastPowerShotA = drive.trajectoryBuilder(shootSecondPowerShotA.end()) // Shoots last power shot
                .lineToConstantHeading(new Vector2d(POWER_SHOT_SHOOT_1.getX(), POWER_SHOT_SHOOT_1.getY() - DISTANCE_BETWEEN_POWER_SHOTS * 2))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot last power shot
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // begin rotating wobble arm ahead of time to save time
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow ring bucket to be dropped
                    }
                })
                .build();

        Trajectory deliverWobbleGoal1A = drive.trajectoryBuilder(shootLastPowerShotA.end()) // Drives to Target Zone A and delivers first wobble goal
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // begin dropping ring elevator
                        ringShooter.setPower(0); // turn flywheel off
                    }
                })
                .lineToLinearHeading(TARGET_ZONE_A1)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION); // release and score first wobble goal
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // retract finger servo to prevent accidental damage
                        pause(500);
                    }
                })
                .build();

        Trajectory backFromWobbleGoal1A = drive.trajectoryBuilder(deliverWobbleGoal1A.end()) // Backs up from the first wobble goal to allow resetting of the arm
                .lineToLinearHeading(new Pose2d(TARGET_ZONE_A1.getX(), TARGET_ZONE_A1.getY() + 14, TARGET_ZONE_A1.getHeading() - Math.toRadians(45)))
                .build();

        Trajectory driveToCollectWobbleGoal2A = drive.trajectoryBuilder(backFromWobbleGoal1A.end()) // Aligns to collect second wobble goal
                .lineToLinearHeading(SECOND_WOBBLE_GOAL_ALIGN_POSITION)
                .build();

        Trajectory collectWobbleGoal2A = drive.trajectoryBuilder(driveToCollectWobbleGoal2A.end()) // Drives to and collects second wobble goal for scoring
                .lineToLinearHeading(SECOND_WOBBLE_GOAL_PICKUP_POSITION_A)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION); // close wobble claw to grasp second wobble goal
                        pause(500); // pause to allow wobble claw time to fully close
                        wobbleArm.setTargetPosition(ARM_HOVER_POSITION); // raise wobble goal enough so that it doesn't drag on the field floor
                        pause(500); // pause to allow time to lift the wobble goal off of the floor to prevent drag
                    }
                })
                .build();

        Trajectory deliverWobbleGoal2A = drive.trajectoryBuilder(collectWobbleGoal2A.end()) // Drives to target zone A to deliver the second wobble goal
                .lineToLinearHeading(new Pose2d(TARGET_ZONE_A2.getX(), TARGET_ZONE_A2.getY(), TARGET_ZONE_A2.getHeading()))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // drop wobble arm to set wobble goal on floor
                        pause(500); // allow wobble arm time to rotate
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION);
                        pause(300); // allow claw time to open and release wobble goal
                    }
                })
                .build();

        Trajectory backFromWobbleGoal2A = drive.trajectoryBuilder(deliverWobbleGoal2A.end()) // Back from the scored wobble goals to prevent tipping them before parking
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_STARTING_POSITION); // raise wobble arm to prevent accidental damage
                    }
                })
                .lineToLinearHeading(new Pose2d(TARGET_ZONE_A2.getX(), TARGET_ZONE_A2.getY() + 14, TARGET_ZONE_A2.getHeading() - Math.toRadians(45)))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION); // close wobble claw to prevent accidental damage
                    }
                })
                .build();

        Trajectory parkA = drive.trajectoryBuilder(backFromWobbleGoal2A.end()) // Navigate to launch line
                .lineToLinearHeading(PARKING_POSE)
                .build();


// Case B Trajectories
        Trajectory shootStartingRingsB = drive.trajectoryBuilder(STARTING_POSE) // Drives to shooting pose while initialising necessary motors
                .lineToLinearHeading(LONG_SHOT_POSE) // Approach starter stack to score in high goal
                .addSpatialMarker(new Vector2d(-58, -32), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // set finger servo to out position
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring bucket
                        ringShooter.setVelocity(highGoalTPS); // set flywheel speed to score in high goal
                    }
                })
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500); // allow ring elevator time to raise before shooting starting rings
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                        pause(500); // allow time to shoot ring
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo to allow ring elevator to drop
                        pause(300); // allow time for finger servo to move
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake starter stack rings
                        ringShooter.setPower(0); // Turn off flywheel
                        pause(500); // allow time for ring elevator to drop
                    }
                })
                .build();

        Trajectory collectStarterStackRingsB = drive.trajectoryBuilder(shootStartingRingsB.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_IN_POWER); // power intake to collect starter stack rings
                    }
                })
                .lineToLinearHeading(new Pose2d(STARTER_STACK.getX(), STARTER_STACK.getY(), Math.toRadians(0)))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1500); // allow time for intake to collect ring
                    }
                })
                .build();

        Trajectory toPowerShotsB = drive.trajectoryBuilder(collectStarterStackRingsB.end()) // Drives to and scores first power shots while initialising necessary motors
                .lineToLinearHeading(POWER_SHOT_SHOOT_1)
                .addSpatialMarker(new Vector2d(-30, -30), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // set finger servo to out position
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring bucket
                        ringShooter.setVelocity(powerShotTPS); // spin up flywheel to score power shots
                        intakeDrive.setPower(0); // power intake off
                    }
                })
                .addDisplacementMarker(new MarkerCallback() {  // When destination reached, shoot first power shot
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot first power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootSecondPowerShotB = drive.trajectoryBuilder(toPowerShotsB.end()) // Shoots second power shot
                .lineToConstantHeading(new Vector2d(POWER_SHOT_SHOOT_1.getX(), POWER_SHOT_SHOOT_1.getY() - DISTANCE_BETWEEN_POWER_SHOTS))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot second power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootLastPowerShotB = drive.trajectoryBuilder(shootSecondPowerShotB.end()) // Shoot last power shot
                .lineToConstantHeading(new Vector2d(POWER_SHOT_SHOOT_1.getX(), POWER_SHOT_SHOOT_1.getY() - DISTANCE_BETWEEN_POWER_SHOTS * 2))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot last power shot
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // begin rotating wobble arm ahead of time to save time
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow ring bucket to be dropped
                    }
                })
                .build();

        Trajectory deliverWobbleGoal1B = drive.trajectoryBuilder(shootLastPowerShotB.end()) // Drives to Target Zone B and delivers first wobble goal
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // begin dropping ring elevator
                        ringShooter.setPower(0); // turn flywheel off
                    }
                })
                .lineToLinearHeading(TARGET_ZONE_B1)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION); // release and score first wobble goal
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // retract finger servo to prevent accidental damage
                        pause(500);
                    }
                })
                .build();

        Trajectory backFromWobbleGoal1B = drive.trajectoryBuilder(deliverWobbleGoal1B.end()) // Backs up from the first wobble goal to avoid tipping it
                .lineToLinearHeading(new Pose2d(TARGET_ZONE_B1.getX() - 14, TARGET_ZONE_B1.getY(), TARGET_ZONE_B1.getHeading() - Math.toRadians(45)))
                .build();

        Trajectory driveToCollectWobbleGoal2B = drive.trajectoryBuilder(backFromWobbleGoal1B.end()) // Aligns to collect second wobble goal
                .lineToLinearHeading(SECOND_WOBBLE_GOAL_ALIGN_POSITION)
                .build();

        Trajectory collectWobbleGoal2B = drive.trajectoryBuilder(driveToCollectWobbleGoal2B.end()) // Drives to and collects second wobble goal for scoring
                .lineToLinearHeading(SECOND_WOBBLE_GOAL_PICKUP_POSITION_B)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION); // close wobble claw to grasp second wobble goal
                        pause(500); // pause to allow wobble claw time to fully close
                        wobbleArm.setTargetPosition(ARM_HOVER_POSITION); // raise wobble goal enough so that it doesn't drag on the field floor
                        pause(500); // pause to allow time to lift the wobble goal off of the floor to prevent drag
                    }
                })
                .build();

        Trajectory deliverWobbleGoal2B = drive.trajectoryBuilder(collectWobbleGoal2B.end()) // Drives to target zone B to deliver the second wobble goal
                .lineToLinearHeading(TARGET_ZONE_B2)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // drop wobble arm to set wobble goal on floor
                        pause(500); // allow wobble arm time to rotate
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION);
                        pause(300); // allow claw time to open and release wobble goal
                    }
                })
                .build();

        Trajectory backFromWobbleGoal2B = drive.trajectoryBuilder(deliverWobbleGoal2B.end()) // Back from the scored wobble goals before parking
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_STARTING_POSITION); // raise wobble arm to prevent accidental damage
                    }
                })
                .lineToLinearHeading(new Pose2d(TARGET_ZONE_B2.getX() - 10, TARGET_ZONE_B2.getY(), TARGET_ZONE_B2.getHeading() - Math.toRadians(45)))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION); // close wobble claw to prevent accidental damage
                    }
                })
                .build();

        Trajectory parkB = drive.trajectoryBuilder(backFromWobbleGoal2B.end()) // Navigate to launch line
                .lineToLinearHeading(PARKING_POSE)
                .build();


// Case C Trajectories
        /*Trajectory shootStartingRingsC = drive.trajectoryBuilder(STARTING_POSE) // Drives shooting pose while initialising necessary motors
                .lineToLinearHeading(LONG_SHOT_POSE) // Approach starter stack to score in high goal
                .addSpatialMarker(new Vector2d(-58, -32), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // set finger servo to out position
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring bucket
                        ringShooter.setVelocity(highGoalTPS); // set flywheel speed to score in high goal
                    }
                })
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500); // allow ring elevator time to raise before shooting starting rings
                        for (byte i = 1; i <= 3; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot ring
                            pause(500); // allow time to shoot ring
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo to allow next ring to drop
                            pause(500); // allow time for finger servo to move
                        }
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake starter stack rings
                        ringShooter.setPower(0); // Turn off flywheel
                        pause(500); // allow time for ring elevator to drop
                    }
                })
                .build();

        Trajectory displaceStarterStackRingsC = drive.trajectoryBuilder(shootStartingRingsC.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_OUT_POWER); // power intake to displace starter stack rings
                    }
                })
                .lineToLinearHeading(new Pose2d(STARTER_STACK.getX(), STARTER_STACK.getY(), Math.toRadians(0)))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000); // allow time for intake to collect ring
                    }
                })
                .build();

        Trajectory collectStarterStackRings1C = drive.trajectoryBuilder(displaceStarterStackRingsC.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_IN_POWER); // power intake to collect the rest of the starter stack rings
                    }
                })
                .lineToLinearHeading(new Pose2d(STARTER_STACK.getX() + 12, STARTER_STACK.getY(), 0), // slowly intake rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000); // allow time for intake to pull rings into ring elevator
                    }
                })
                .build();
        
        Trajectory shootStarterStackRingsC = drive.trajectoryBuilder(collectStarterStackRings1C.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringShooter.setVelocity(highGoalTPS); // spin up ring shooter to shoot into high goal
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo out to avoid damage 
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to shoot rings
                        intakeDrive.setPower(0); // turn off intake
                    }
                })
                .lineToLinearHeading(LONG_SHOT_POSE) // navigate back to high goal shooting position
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                        pause(500); // allow time to shoot ring
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo to out position to allow ring elevator to be dropped
                        pause(300);
                    }
                })
                .build();

        Trajectory collectStarterStackRings2C = drive.trajectoryBuilder(shootStarterStackRingsC.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        intakeDrive.setPower(INTAKE_IN_POWER); // power intake to collect rings
                    }
                })
                .lineToLinearHeading(new Pose2d(STARTER_STACK.getX() + 24, STARTER_STACK.getY(), 0), // slowly intake rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000); // allow intake time to feed rings into ring elevator
                    }
                })
                .build();*/

        Trajectory toPowerShotsC = drive.trajectoryBuilder(STARTING_POSE) // Drives to and scores first power shots while initialising necessary motors
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(0);
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // set finger servo to out position
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring bucket
                        ringShooter.setVelocity(powerShotTPS); // spin up flywheel to score power shots
                    }
                })
                .lineToLinearHeading(POWER_SHOT_SHOOT_1)
                .addDisplacementMarker(new MarkerCallback() {  // When destination reached, shoot first power shot
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot first power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootSecondPowerShotC = drive.trajectoryBuilder(toPowerShotsC.end()) // Shoots second power shot
                .lineToConstantHeading(new Vector2d(POWER_SHOT_SHOOT_1.getX(), POWER_SHOT_SHOOT_1.getY() - DISTANCE_BETWEEN_POWER_SHOTS))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot second power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow rings on top to drop into place
                    }
                })
                .build();

        Trajectory shootLastPowerShotC = drive.trajectoryBuilder(shootSecondPowerShotC.end()) // Shoot last power shot
                .lineToConstantHeading(new Vector2d(POWER_SHOT_SHOOT_1.getX(), POWER_SHOT_SHOOT_1.getY() - DISTANCE_BETWEEN_POWER_SHOTS * 2))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot last power shot
                        pause(500); // pause to allow servo to move
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger to allow ring bucket to be dropped
                    }
                })
                .build();

        Trajectory deliverWobbleGoal1C = drive.trajectoryBuilder(shootLastPowerShotC.end()) // Drives to Target Zone A and delivers first wobble goal
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringShooter.setPower(0); // turn flywheel off
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // begin dropping ring elevator
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // begin rotating wobble arm ahead of time to save time
                    }
                })
                .lineToLinearHeading(TARGET_ZONE_C1)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION); // release and score first wobble goal
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // retract finger servo to prevent accidental damage
                        pause(500);
                    }
                })
                .build();

        Trajectory backFromWobbleGoal1C = drive.trajectoryBuilder(deliverWobbleGoal1C.end()) // Backs up from the first wobble goal to avoid tipping it
                .lineToLinearHeading(new Pose2d(TARGET_ZONE_C1.getX() - 14, TARGET_ZONE_C1.getY() + 5, Math.toRadians(135)))
                .build();

        Trajectory driveToCollectWobbleGoal2C = drive.trajectoryBuilder(backFromWobbleGoal1C.end()) // Aligns to collect second wobble goal
                .lineToLinearHeading(SECOND_WOBBLE_GOAL_ALIGN_POSITION_C)
                .build();

        Trajectory collectWobbleGoal2C = drive.trajectoryBuilder(driveToCollectWobbleGoal2C.end()) // Drives to and collects second wobble goal for scoring
                .lineToLinearHeading(SECOND_WOBBLE_GOAL_PICKUP_POSITION_C)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION); // close wobble claw to grasp second wobble goal
                        pause(500); // pause to allow wobble claw time to fully close
                        wobbleArm.setTargetPosition(ARM_HOVER_POSITION); // raise wobble goal enough so that it doesn't drag on the field floor
                        pause(500); // pause to allow time to lift the wobble goal off of the floor to prevent drag
                    }
                })
                .build();

        Trajectory deliverWobbleGoal2C = drive.trajectoryBuilder(collectWobbleGoal2C.end()) // Drives to target zone C to deliver the second wobble goal
                .lineToLinearHeading(TARGET_ZONE_C2)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_DOWN_POSITION); // drop wobble arm to set wobble goal on floor
                        pause(500); // allow wobble arm time to rotate
                        wobbleClaw.setPosition(CLAW_OPENED_POSITION);
                        pause(300); // allow claw time to open and release wobble goal
                    }
                })
                .build();

        Trajectory backFromWobbleGoal2C = drive.trajectoryBuilder(deliverWobbleGoal2C.end()) // Back from the scored wobble goals before parking
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleArm.setTargetPosition(ARM_STARTING_POSITION); // raise wobble arm to prevent accidental damage
                    }
                })
                .lineToLinearHeading(new Pose2d(TARGET_ZONE_C2.getX() - 10, TARGET_ZONE_C2.getY(), Math.toRadians(135)))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobbleClaw.setPosition(CLAW_CLOSED_POSITION); // close wobble claw to prevent accidental damage
                    }
                })
                .build();

        Trajectory parkC = drive.trajectoryBuilder(backFromWobbleGoal2C.end()) // Navigate to launch line
                .lineToLinearHeading(PARKING_POSE)
                .build();




// INITIALIZE HARDWARE:
        // Initialize intake motor
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

        // Initialize flywheel motor
        ringShooter = hardwareMap.get(DcMotorEx.class, "shoot");

        // Initialize Wobble Goal manipulation motors
        wobbleClaw = hardwareMap.get(Servo.class, "WGClaw");
        wobbleArm = hardwareMap.get(DcMotor.class, "WGArm");
        fingerServo = hardwareMap.get(Servo.class, "ringFinger");

        // Initialize ring elevator motor
        ringElevator = hardwareMap.get(DcMotor.class, "ringElevator");

        // Initialize motor and servo Positions
        // Set Motor Directions
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set servo initialization positions
        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);

        // Apply motor power
        // Apply power and set mode for wobble arm motor
        wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
        wobbleArm.setPower(.5);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power and set mode for ring elevator motor
        ringElevator.setTargetPosition(ringElevator.getCurrentPosition());
        ringElevator.setPower(0.8);
        ringElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run mode

        // Obtain encoder positions based on starting position and deltas
        // Obtain wobble arm position values
        ARM_STARTING_POSITION = wobbleArm.getCurrentPosition() + 20;  // Get the starting position of the arm based on the current position of the arm at init time, which is assumed.
        ARM_UP_POSITION = wobbleArm.getCurrentPosition() - ARM_UP_POSITION_DELTA;  // Get the up position of the arm with respect to the current position of the arm at init time, which is assumed.
        ARM_DOWN_POSITION = wobbleArm.getCurrentPosition() - ARM_DOWN_POSITION_DELTA;  // Get the down position of the arm with respect to the current position of the arm at init time, which is assumed.
        ARM_HOVER_POSITION = wobbleArm.getCurrentPosition() - ARM_HOVER_POSITION_DELTA;  // Get the hover position of the arm with respect to the current position of the arm at init time, which is assumed.

        // Obtain ring elevator position values
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2017;

        // Set flywheel PIDF coefficients
        ringShooter.setVelocityPIDFCoefficients(150, 7, 10, 0);

// WAIT FOR AUTONOMOUS TO BEGIN:
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        if (autoCase == 'A') { // follow paths for autonomous case A
            drive.followTrajectory(toPowerShotsA);
            drive.followTrajectory(shootSecondPowerShotA);
            drive.followTrajectory(shootLastPowerShotA);
            drive.followTrajectory(deliverWobbleGoal1A);
            drive.followTrajectory(backFromWobbleGoal1A);
            drive.followTrajectory(driveToCollectWobbleGoal2A);
            drive.followTrajectory(collectWobbleGoal2A);
            drive.followTrajectory(deliverWobbleGoal2A);
            drive.followTrajectory(backFromWobbleGoal2A);
            drive.followTrajectory(parkA);
        }
        else if (autoCase == 'B') { // follow paths for autonomous case B
            drive.followTrajectory(shootStartingRingsB);
            drive.followTrajectory(collectStarterStackRingsB);
            drive.followTrajectory(toPowerShotsB);
            drive.followTrajectory(shootSecondPowerShotB);
            drive.followTrajectory(shootLastPowerShotB);
            drive.followTrajectory(deliverWobbleGoal1B);
            drive.followTrajectory(backFromWobbleGoal1B);
            drive.followTrajectory(driveToCollectWobbleGoal2B);
            drive.followTrajectory(collectWobbleGoal2B);
            drive.followTrajectory(deliverWobbleGoal2B);
            drive.followTrajectory(backFromWobbleGoal2B);
            drive.followTrajectory(parkB);
        }
        else if (autoCase == 'C') { // follow paths for autonomous case C
            /*drive.followTrajectory(shootStartingRingsC);
            drive.followTrajectory(displaceStarterStackRingsC);
            drive.followTrajectory(collectStarterStackRings1C);
            drive.followTrajectory(shootStarterStackRingsC);
            drive.followTrajectory(collectStarterStackRings2C);*/
            drive.followTrajectory(toPowerShotsC);
            drive.followTrajectory(shootSecondPowerShotC);
            drive.followTrajectory(shootLastPowerShotC);
            drive.followTrajectory(deliverWobbleGoal1C);
            drive.followTrajectory(backFromWobbleGoal1C);
            drive.followTrajectory(driveToCollectWobbleGoal2C);
            drive.followTrajectory(collectWobbleGoal2C);
            drive.followTrajectory(deliverWobbleGoal2C);
            drive.followTrajectory(backFromWobbleGoal2C);
            drive.followTrajectory(parkC);
        }
        else telemetry.addLine("Error: Check auto case determination in program."); // debugging message
    }

    void pause(long waitTime) { // pause function to prevent new commands from being executed
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + waitTime) {}
    }

    class imageFeedPipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        Mat imageCrop = new Mat();
        Mat imageHSV = new Mat();
        Mat ringMask = new Mat();
        int ringPixels = 0;
        double[] pixel;
        double[] setpixel;
        Point setpixelPoint;
        double redValue, greenValue, blueValue;
        final int resolutionTuner = 5; // One pixel sampled every # pixels.  Raise for speed, lower for reliability.
        final int RED_VALUE_MIN = 100;
        final double ORANGE_GB_LOW_THRESHOLD = 1.5;
        final double ORANGE_RG_LOW_THRESHOLD = 1.25;
        final double ORANGE_RB_LOW_THRESHOLD = 5.0;
        public int totalPixels = 0;


        @Override
        public Mat processFrame(Mat imageFeed) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            final int RING_SECTION_CROP_Y1 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENT_Y1);
            final int RING_SECTION_CROP_Y2 = (int) (imageFeed.rows() * RING_SCAN_CROP_PERCENT_Y2);
            final int RING_SECTION_CROP_X1 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENT_X1);
            final int RING_SECTION_CROP_X2 = (int) (imageFeed.cols() * RING_SCAN_CROP_PERCENT_X2);
            final Scalar ringVisualizeColor = new Scalar(0.0d, 255.0d, 0.0d);

            setpixelPoint = new Point(0, 0);

            totalPixels = ((RING_SECTION_CROP_Y2 - RING_SECTION_CROP_Y1) * (RING_SECTION_CROP_X2 - RING_SECTION_CROP_X1) / resolutionTuner);

            ringPixels = 0;
            for(int x = RING_SECTION_CROP_X1; x <= RING_SECTION_CROP_X2; x += resolutionTuner) {
                for (int y = RING_SECTION_CROP_Y1; y <= RING_SECTION_CROP_Y2; y += resolutionTuner) {
                    pixel = imageFeed.get(y, x);
                    redValue = pixel[0];
                    greenValue = pixel[1];
                    blueValue = pixel[2];
                    if (/*redValue >= blueValue * ORANGE_RB_LOW_THRESHOLD && */redValue >= greenValue * ORANGE_RG_LOW_THRESHOLD && greenValue >= blueValue * ORANGE_GB_LOW_THRESHOLD) {
                        ringPixels++;
                        // imageFeed.set(y, x, setpixel)
                        setpixelPoint.x = (int) x;
                        setpixelPoint.y = (int) y;
                        Imgproc.circle(imageFeed, setpixelPoint, resolutionTuner / 2, ringVisualizeColor, Imgproc.FILLED);
                    }
                }
            }

            Imgproc.rectangle(
                    imageFeed,
                    new Point(
                            RING_SECTION_CROP_X1,
                            RING_SECTION_CROP_Y1),
                    new Point(
                            RING_SECTION_CROP_X2,
                            RING_SECTION_CROP_Y2),
                    new Scalar(0, 255, 0), 4);


            ringImagePercent = (double) ringPixels / ((double) totalPixels / resolutionTuner);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

             /*
              * Send some stats to the telemetry
              */

            char autoCase_ = 'X';

            if (ringImagePercent >= oneRingPercentageMinimum && ringImagePercent < fourRingPercentageMinimum) { autoCase = 'B'; }
            else if (ringImagePercent >= fourRingPercentageMinimum) { autoCase = 'C'; }
            else {autoCase = 'A'; }

            int pixels = pipeline.getRingPixels();
            telemetry.addData("Auto Case: ", (char) (autoCase));
            telemetry.addData("Ring Percentage: ", ringImagePercent);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Ring-Colored Pixels: ", pixels);
            telemetry.addData("Ring crop region X1", RING_SCAN_CROP_PERCENT_X1);
            telemetry.addData("Ring crop region Y1", RING_SCAN_CROP_PERCENT_Y1);
            telemetry.addData("Ring crop region X2", RING_SCAN_CROP_PERCENT_X2);
            telemetry.addData("Ring crop region Y2", RING_SCAN_CROP_PERCENT_Y2);
            telemetry.addData("Total pixels", pipeline.totalPixels);
            telemetry.update();

            return imageFeed;
        }

        public int getRingPixels() {
            return ringPixels;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
