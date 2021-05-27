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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class TNWRingAuto extends LinearOpMode {

    // ROADRUNNER VALUES:
    // Constant Roadrunner Pose Values
    // Starting poses
    final Pose2d STARTING_POSE_INNER = new Pose2d(-63, -18, Math.toRadians(0)); // TODO: set this to allow more room for alliance partner and elements
    final Pose2d STARTING_POSE_OUTER = new Pose2d(-63, -(56 + 3.0/8), Math.toRadians(0)); // TODO: set these

    // Alliance Partner Element pickup positions
    Pose2d PARTNER_RINGS_POSITION; // used by the autonomous. to be set in initializeElementPositions() function
    Pose2d PARTNER_RINGS_POSITION_INNER = new Pose2d(-58, -32, Math.toRadians(90));
    Pose2d PARTNER_RINGS_POSITION_OUTER = new Pose2d(-58, -34, Math.toRadians(90));

    // Starter stack related poses
    final Vector2d STARTER_STACK = new Vector2d(-24, -35);
    final Pose2d LONG_SHOT_POSE = new Pose2d(-40, -37, Math.toRadians(355)); // y = -36, Heading = 356
    // Power shot related poses
    //final Pose2d POWER_SHOT_SHOOT_1 = new Pose2d(-3, -3.5, Math.toRadians(356));
    //final double DISTANCE_BETWEEN_POWER_SHOTS = 8; // inches
    // Parking pose
    //final Pose2d PARKING_POSE = new Pose2d(12, -24, Math.toRadians(0));

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
    private DcMotorEx ringElevator;

    // MOTOR AND SERVO POSITION CONSTANTS:
    private static double highGoalTPS = 56.5 * 28; // 57.5  // Ticks per second of the shooter when active and aiming for the high goal
    private static double powerShotTPS = 50 * 28; // Ticks per second of the shooter when active and aiming for the power shots

    final int RING_SHOOT_TIME = 750; // ring scoring interval measured in milliseconds

    private static double CLAW_OPENED_POSITION = 0.0;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.48;  // The position of the claw when it is closed

    private static int ARM_DOWN_POSITION_DELTA = 422;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's down
    private static int ARM_UP_POSITION_DELTA = 222;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up
    private static int ARM_HOVER_POSITION_DELTA = 330;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up

    private static int ARM_DOWN_POSITION;  // The absolute position (in motor encoder units) of the arm's down position. Set on init
    private static int ARM_UP_POSITION;  // The absolute position (in motor encoder units) of the arm's up position. Set on init
    private static int ARM_HOVER_POSITION;  // The absolute position (in motor encoder units) of the arm's hover position. Set on init
    private static int ARM_STARTING_POSITION; // The absolute position (in motor encoder units) of the arm's starting position. Set on init

    private static int RING_ELEVATOR_UP_POSITION;  // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION;  // The position of the Ring Elevator when it is in the DOWN state
    private static double RING_ELEVATOR_POWER = 0.7;  // The power for the motor to use when running to its target position

    private static double RING_FINGER_IN_POSITION = 0.12;  // The position of the ring finger when it's in
    private static double RING_FINGER_OUT_POSITION = 0.50;  // The position of the ring finger when it's out

    private static double INTAKE_IN_POWER = -0.85;  // The power set to intake drive for collecting rings
    private static double INTAKE_OUT_POWER = INTAKE_IN_POWER * -1;  // The power set to intake drive for ejecting rings

    private long autoStartTime;

    // Gamepad inputs
    private boolean gamepad1AHeld = false;  // Whether or not the gamepad 1 a button is being held, handled by the handleInput function
    private boolean gamepad1APressed = false;  // Whether or not the gamepad 1 a button was JUST pressed, handled by the handleInput function
    private boolean gamepad1BHeld = false;  // Whether or not the gamepad 1 b button is being held, handled by the handleInput function
    private boolean gamepad1BPressed = false;  // // Whether or not the gamepad 1 b button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadLeftHeld = false;  // Whether or not the gamepad 1 dpad left button is being held, handled by the handleInput function
    private boolean gamepad1DpadRightHeld = false;  // Whether or not the gamepad 1 dpad right button is being held, handled by the handleInput function
    private boolean gamepad1DpadLeftPressed = false;  // Whether or not the gamepad 1 dpad left button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadRightPressed = false;  // Whether or not the gamepad 1 dpad right button was JUST pressed, handled by the handleInput function


    // Driver Input Variables
    boolean nextStep = false; // if nextStep is true, driver input will continue to the next step
    boolean blueAlliance = false; // are we on the blue alliance?  indicated by drivers in init
    byte startingPosition = 2; // starting position indicated by drivers in init
    boolean scoreAlliancePartnerWobble = true; // whether to collect and deliver alliance partner's wobble goal.  indicated by drivers in configuration
    boolean scoreAlliancePartnerRings = true;  // whether to collect and score alliance partner's preloaded rings.  indicated by drivers in configuration
    boolean deliverWobble = true; // whether to deliver preloaded wobble goal.  indicated by drivers in configuration
    boolean navigateToLaunchLine = true; // whether or not to park on the launch line at the end of the autonomous period.  indicated by drivers in configuration
    byte parkingLocation = 2; // field tiles from alliance wall to park indicated by drivers in configuration
    boolean alliancePartnerMoves = false;
    boolean buttonsReleased = true; // gamepad buttons are released before triggering queues

    SampleMecanumDrive drive;

    // FUNCTIONS:

    private void handleInput() {
        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;  // Whether or not the gamepad 1 a button was held
        boolean gamepad1BWasHeld = gamepad1BHeld;  // Whether or not the gamepad 1 b button was held
        boolean gamepad1DpadLeftWasHeld = gamepad1DpadLeftHeld;  // Whether or not the gamepad 1 dpad left button was held
        boolean gamepad1DpadRightWasHeld = gamepad1DpadRightHeld;  // Whether or not the gamepad 1 dpad right button was held

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1BHeld = gamepad1.b;
        gamepad1DpadLeftHeld = gamepad1.dpad_left;
        gamepad1DpadRightHeld = gamepad1.dpad_right;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1DpadLeftPressed = !gamepad1DpadLeftWasHeld && gamepad1DpadLeftHeld;
        gamepad1DpadRightPressed = !gamepad1DpadRightWasHeld && gamepad1DpadRightHeld;
    }

    private void initializeElementPositions() {
        switch (startingPosition) {
            case 1: PARTNER_RINGS_POSITION = PARTNER_RINGS_POSITION_OUTER; break;
            case 2: PARTNER_RINGS_POSITION = PARTNER_RINGS_POSITION_INNER; break;
        }
    }

    Pose2d selInvertPose(Pose2d inputPose) {
        if (blueAlliance) {
            return new Pose2d(inputPose.getX(), -inputPose.getY(), Math.toRadians(360) - inputPose.getHeading());
        } else {
            return inputPose;
        }
    }

    Pose2d selInvertPose(Pose2d inputPose, boolean universalHeading) {
        if (blueAlliance) {
            if (universalHeading)
                return new Pose2d(inputPose.getX(), -inputPose.getY(), inputPose.getHeading());
            else
                return new Pose2d(inputPose.getX(), -inputPose.getY(), Math.toRadians(360) - inputPose.getHeading());
        } else {
            return inputPose;
        }
    }

    Vector2d selInvertPose(Vector2d inputVector) {
        if (blueAlliance) {
            return new Vector2d(inputVector.getX(), -inputVector.getY());
        } else {
            return inputVector;
        }
    }

    double selInvertPose(double inputEndTangent) {
        if (blueAlliance) return Math.toRadians(360) - inputEndTangent;
        else return inputEndTangent;
    }

    private void stopMillis(long delay) {
        // Stops the robot for `delay` milliseconds
        // Assumes that a SampleMecanumDrive with a public mode variable and Mode enum is in scope
        drive.mode = SampleMecanumDrive.Mode.IDLE;
        long startTime = System.currentTimeMillis();

        drive.setMotorPowers(0, 0, 0, 0);

        while (System.currentTimeMillis() - startTime < delay) {  // Assumes we're running in an iterative OpMode where blocking is permitted
            drive.update();  // Keep track of PID, but IDLE Mode makes it try to stop the bot
        }

        drive.mode = SampleMecanumDrive.Mode.FOLLOW_TRAJECTORY;  // Return to path following
    }

    private void pause(long millis) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + millis) {
            stopMillis(1);
            ringShooter.setVelocity(highGoalTPS);
        }
    }

    private void pause(long millis, boolean flywheel) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + millis) {
            stopMillis(1);
            if (flywheel) ringShooter.setVelocity(highGoalTPS);
        }
    }

    private void pause(long millis, double TPS) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() <= startTime + millis) {
            stopMillis(1);
            ringShooter.setVelocity(TPS);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // obtain alliance color info from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    blueAlliance = false;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    blueAlliance = true;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Red Alliance? (A for yes, B for no)");
                telemetry.update();
            }
        }
        // obtain starting line position from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    startingPosition = 1;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    startingPosition = 2;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Starting Position? (A for inner, B for outer)");
                telemetry.update();
            }
        }
        // obtain whether to score alliance partner rings from drivers
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    scoreAlliancePartnerRings = true;
                    nextStep = true;
                } else if (gamepad1BPressed) {
                    scoreAlliancePartnerRings = false;
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Score alliance partner's rings? (A for Yes, B for No)");
                telemetry.update();
            }
        }
        // allow driver to review selected configuration
        nextStep = false;
        buttonsReleased = false;
        while (!nextStep) {
            handleInput();
            if (!gamepad1APressed && !gamepad1BPressed) buttonsReleased = true;
            if (buttonsReleased) {
                if (gamepad1APressed) {
                    nextStep = true;
                }
                telemetry.addData("Obtaining ", "Reviewing Selections (A to confirm)");
                if (blueAlliance) telemetry.addData("Alliance ", "BLUE");
                else telemetry.addData("Alliance ", "RED");
                if (startingPosition == 1) telemetry.addData("Starting Line ", "INNER");
                else telemetry.addData("Starting Line ", "OUTER");
                if (scoreAlliancePartnerRings) telemetry.addLine("Score Alliance Partner Rings");
                else telemetry.addLine("Don't Score Alliance Partner Rings");
                telemetry.update();
            }
        }

        drive.setPoseEstimate(selInvertPose(startingPosition == 1 ? STARTING_POSE_INNER : STARTING_POSE_OUTER));

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
        ringElevator = hardwareMap.get(DcMotorEx.class, "ringElevator");

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

        // Obtain ring elevator position values
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2000; // 2017

        // Set DcMotorEx PIDF coefficients
        ringShooter.setVelocityPIDFCoefficients(150, 7, 10, 0);
        ringElevator.setVelocityPIDFCoefficients(5, 3, 3, 0);

        // Initialize Element Positions
        initializeElementPositions();

        // Build Roadrunner Trajectories

        Trajectory scorePreloadedRings = drive.trajectoryBuilder(selInvertPose(startingPosition == 1 ? STARTING_POSE_INNER : STARTING_POSE_OUTER)) // drive from the start pose to the starter stack and shoot preloaded rings
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo out to make room to avoid collision with ring elevator
                        ringShooter.setVelocity(57.75 * 28); // spin up ring shooter to score in high goal // using a different speed since we are farther back.
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to drop intake shield
                    }
                })
                .splineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX(), LONG_SHOT_POSE.getY(), LONG_SHOT_POSE.getHeading()), true), selInvertPose(Math.toRadians(startingPosition == 1 ? 270 : 90)))
                // a series of temporal markers is preferred over a looped sequence with pauses to preserve roadrunner PID accuracy
                .addTemporalMarker(.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(0); // turn off intake
                    }
                })
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500); // allow flywheel PID to adjust
                        for (int i = 1; i <= 4; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2, 57.75 * 28);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2, 57.75 * 28);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory alignToCollectPartnerRings = drive.trajectoryBuilder(scorePreloadedRings.end())
                .splineToLinearHeading(selInvertPose(new Pose2d(PARTNER_RINGS_POSITION.getX(), startingPosition == 2 ? PARTNER_RINGS_POSITION.getY() - 25 : PARTNER_RINGS_POSITION.getY() + 25, Math.toRadians(startingPosition == 2 ? 90 : 270))), selInvertPose(Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI / 2, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_IN_POSITION); // retract finger servo into robot to avoid accidental damage
                    }
                })
                .build();

        Trajectory collectAlliancePartnerRings = drive.trajectoryBuilder(alignToCollectPartnerRings.end())
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        intakeDrive.setPower(INTAKE_IN_POWER); // power on intake to collect alliance partner's rings
                    }
                })
                .lineToConstantHeading(selInvertPose(new Vector2d(PARTNER_RINGS_POSITION.getX(), startingPosition == 2 ? PARTNER_RINGS_POSITION.getY() + 10 : PARTNER_RINGS_POSITION.getY() - 10)),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500, false);
                    }
                })
                .build();

        Trajectory shootAlliancePartnerRings = drive.trajectoryBuilder(collectAlliancePartnerRings.end())
                .addTemporalMarker(.5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move finger servo to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        intakeDrive.setPower(0); // turn off intake
                        ringShooter.setVelocity(highGoalTPS); // spin up flywheel to score in high goal
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX(), LONG_SHOT_POSE.getY(), LONG_SHOT_POSE.getHeading()), true)) // x + 26 previously
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500);
                        for (int i = 1; i <= 4; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory shootStarterStackRings1 = drive.trajectoryBuilder(scoreAlliancePartnerRings ? shootAlliancePartnerRings.end() : scorePreloadedRings.end()) // only used in auto cases B and C
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500, false);
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to collect starter stack rings
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 13, LONG_SHOT_POSE.getY(), Math.toRadians(356)), true), // slowly intake starter stack rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000); // allow rings time to completely enter elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move ring finger to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        ringShooter.setVelocity(highGoalTPS);
                        pause(1750); // allow time for ring elevator to raise
                        intakeDrive.setPower(0); // turn off intake
                        for (int i = 1; i <= 4; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();

        Trajectory shootStarterStackRings2 = drive.trajectoryBuilder(shootStarterStackRings1.end()) // only used in auto cases B and C
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(500, false);
                        intakeDrive.setPower(INTAKE_IN_POWER); // turn on intake to collect starter stack rings
                    }
                })
                .lineToLinearHeading(selInvertPose(new Pose2d(LONG_SHOT_POSE.getX() + 26, LONG_SHOT_POSE.getY(), Math.toRadians(356)), true), // slowly intake starter stack rings to avoid jamming the intake
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        pause(1000); // allow rings time to completely enter elevator
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // move ring finger to out position to avoid collision with ring elevator
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION); // raise ring elevator to score rings
                        ringShooter.setVelocity(highGoalTPS);
                        pause(1750); // allow time for ring elevator to raise
                        intakeDrive.setPower(0); // turn off intake
                        for (int i = 1; i <= 4; i++) {
                            fingerServo.setPosition(RING_FINGER_IN_POSITION); // shoot one ring
                            pause(RING_SHOOT_TIME / 2);
                            fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                            pause(RING_SHOOT_TIME / 2);
                        }
                        fingerServo.setPosition(RING_FINGER_OUT_POSITION); // retract finger servo
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION); // drop ring elevator to intake more rings
                        ringShooter.setPower(0); // power off ring shooter
                    }
                })
                .build();



// WAIT FOR AUTONOMOUS TO BEGIN:
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        // set autonomous start time for pause at end
        autoStartTime = System.currentTimeMillis();

        // Follow Trajectories

        drive.followTrajectory(scorePreloadedRings);
        if (scoreAlliancePartnerRings) {
            drive.followTrajectory(alignToCollectPartnerRings);
            drive.followTrajectory(collectAlliancePartnerRings);
            drive.followTrajectory(shootAlliancePartnerRings);
        }
        drive.followTrajectory(shootStarterStackRings1);
        if (!scoreAlliancePartnerRings) {
            drive.followTrajectory(shootStarterStackRings2);
        }

        wobbleArm.setTargetPosition(ARM_STARTING_POSITION);
        intakeDrive.setPower(0);
        ringShooter.setPower(0);
        pause(autoStartTime + 29900 - System.currentTimeMillis(), false);
    }
}