package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.tnwutil.PoseStorage;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOp99Mark2")

public class TeleOp99Mark2 extends OpMode {
    private static boolean AXIS_MOVEMENT = true;  // Whether or not rotation, horizontal, and vertical movement across the field should be controlled by joystick axes rather than dpad buttons

    private static double MOVEMENT_FACTOR = 1.0;  // Floating-point number from 0 to 1 multiplied by all movement motor powers directly before application. Use this to limit the maximum power for ALL movement-related motors evenly

    private static boolean AUTO_PRIORITY = false;  // Whether or not automatic motor and server controls take priority over manual ones instead of the other way around

    private static double JOYSTICK_INPUT_THRESHOLD = 0.10;  // The global threshold for all joystick axis inputs under which no input will be registered. Also referred to as a deadzone

    private static boolean ENABLE_AUTO_DRIVE = true;  // Whether or not automatic driving should be enabled

    private static boolean AUTO_TPS_SELECT = true;  // Whether or not the TPS should be automatically calculated based on our distance to the selected goal

    private double INTAKE_MAX_POWER = 0.8; // the maximum speed for the intake

    private static boolean USE_VARIABLE_SPEED_CURVES = true;  // Whether or not custom curves for movement-related axis input should be used. If this is false, a linear curve will be used
    private static boolean BUTTONS_CYCLE_SPEED_CURVES = false;  // Only applies if using variable speed curves. If this is true, the driver's gamepad buttons (X and Y) will be able to cycle through custom speed curves. A toggles between in, out, and in-out easings and B selects a function (linear, sine, quad, cubic, quart, quint, expo, and circ in order of "curve sharpness")
    private static boolean LEFT_STICK_RESETS_SPEED_CURVE = true;  // Only applies if using variable speed curves. If this is true, the driver's gamepad joystick left stick will reset the speed curve to the default.
    private static int DEFAULT_SPEED_CURVE = 0;  // Only applies if using variable speed curves. The index of the speed curve to use by default (from 0-7 inclusive), and when reset. See above comments for speed curve list
    private static int DEFAULT_SPEED_CURVE_MODE = 0;  // Only applies if using variable speed curves. The index of the speed curve mode to use by default (from 0-2) inclusive. See above comments for mode list

    private static int highGoalTPS = 55 * 28;  // Ticks per second of the shooter when active and aiming for the high goal
    private static int powerShotTPS = 50 * 28;  // Ticks per second of the shooter when active and aiming for the power shots

    private static boolean FOOLPROOF_IMPOSSIBLE_POSITIONS = false;  // Whether or not servo position combinations which are impossible to reach physically will be prevented through software rather than servo gear-grinding, fire or accidental destruction of other parts. This only works to the point that such positions are predicted and tweaked accurately, and may be disabled under careful operation. TODO: This

    private static double ACCELERATION_CAP = 1.33333;  // The max allowed acceleration of any movement motor in units per second per second. This is included because max acceleration might tip the robot, but use with care as very low accelerations will make the robot sluggish. If the cap is reached, velocity will be incremented at this acceleration rather than the alternative. This value should be set to 1 divided by the number of seconds it should take for the motors to increment to maximum velocity. May be set above 1 for accelerations faster than 1 unit per second per second. 1.33333... (the default value) means it should take 0.75 seconds to go from 0 to full. Set to 0 to disable

    private static boolean MOVEMENT_ROTATION_CORRECTION = true;  // Whether or not we should attempt to adjust the robot's movement based on an accumulated rotation offset, which, if accurately maintained, would allow for rotating the robot without affecting movement from the driver's perspective. Disable this if steering seems to drift clockwise or counterclockwise after some amounts of rotation. DO NOTE THAT THIS OPTION EVADES ACCELERATION CHECKS. It does, however, respect all direct power limitations as well as any internal PID control from roadrunner (if any)
    private static boolean LEFT_SHOULDER_RECALIBRATES_ROTATION = true;  // Whether or not the gamepad 1 left shoulder will reset the pose estimate rotation to the initial pose rotation. Only applies if MOVEMENT_ROTATION_CORRECTION is enabled

    private static boolean NATURAL_CLAW_CONTROL = false;  // Whether or not the manual control for the claw should be a toggle. If this is true, holding the claw button will keep the claw closed, and releasing it will open the claw. If this is false, the claw button will toggle the state of the claw opened or closed
    private static boolean INVERT_NATURAL_CLAW_CONTROL = false;  // Only applies if NATURAL_CLAW_CONTROL is used. If this is true, holding the claw button with keep the claw open rather than closed. Likewise releasing the button will close the claw instead of opening it

    // FIXME: Are these the limits of the actual servo, or are they outside the existing range limit?
    // TODO: Sanity check should check using these if they're accurate

    private static double CLAW_OPENED_POSITION = 0.0;  // The position of the claw when it is open
    private static double CLAW_CLOSED_POSITION = 0.48;  // The position of the claw when it is closed

    private static int ARM_DOWN_POSITION_DELTA = 422;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's down (TODO: Set this value)
    private static int ARM_UP_POSITION_DELTA = 222;  // The delta (offset from the init position of the motor's encoder) position of the arm when it's up (TODO: Set this value)

    private static int ARM_DOWN_POSITION;  // The absolute position (in motor encoder units) of the arm's down position. Set on init
    private static int ARM_UP_POSITION;  // The absolute position (in motor encoder units) of the arm's up position. Set on init

    private static int RING_ELEVATOR_UP_POSITION;  // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION;  // The position of the Ring Elevator when it is in the DOWN state
    private static int RING_ELEVATOR_VELOCITY = 6000;  // The velocity of the motor to use when running to its target position TODO: We might increase this

    private static double RING_FINGER_IN_POSITION = 0.12;  // The position of the ring finger when it's in
    private static double RING_FINGER_OUT_POSITION = 0.50;  // The position of the ring finger when it's out

    private static double FULLAXIS_LEFT_WEIGHT = 0.75;  // The weighting of the left joystick when using fullaxis control, from 0 to 1. This should sum with FULLAXIS_RIGHT_WEIGHT to equal exactly 1
    private static double FULLAXIS_RIGHT_WEIGHT = 0.25;  // The weighting of the right joystick when using fullaxis control, from 0 to 1. This should sum with FULLAXIS_LEFT_WEIGHT to equal 1

    private static double SLOW_MODE_POWER_FACTOR = 0.25;  // The amount multiplied to all motor values when in slow mode

    private static boolean FULLAXIS_CONTROL = true;  // Whether or not fullaxis mode is used. With this enabled, both thumb axes contribute equally (half power maximum on each joystick) to the final robot speed. In this mode, the gamepad 1 triggers are used for rotation, which also yields more movement and thus more overall control

    private static boolean RIGHT_SHOULDER_RESETS_CURRENT_TARGET = true;  // Whether or not the gamepad 1 right shoulder will reset the target pose for the trajectory linked to the current target (the one last followed via gamepad 1 ABXY buttons). Only applies if GAMEPAD_XY_TOGGLES_AUTO_DRIVE is true

    private static double GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE = 0.1;  // The threshold over which any movement axes will immediately take back manual control. Only applies if GAMEPAD_XY_TOGGLES_AUTO_DRIVE is true

    private static double AUTO_DISTANCE_THRESHOLD = 1.0;  // The maximum error (in inches) when the failsafe code will decide that Roadrunner won't error out when trying to construct a spline of this distance. In other words, trajectories shorter than this distance will not be run
    private static double AUTO_ANGULAR_DISTANCE_THRESHOLD = Math.toRadians(5);  // 5 degrees maximum error (same as above, but for rotation)

    private static int PMODE = 0;  // PMODE, for problems

    private long time;  // The current time, used to measure delta time. Set at init and every loop iteration
    private long lastTime;  // The time of the LAST tick, used to measure delta time. Set at init and every loop iteration
    private long deltaTime;  // The delta time between ticks in milliseconds. Set at init and every loop iteration

    private double clawPosition;  // The current target position of the claw. setPosition is called using this value at the very end of the loop, only once
    private double fingerPosition;  // The current target position of the ring finger. setPosition is called using this value at the very end of the loop, only once
    private int armPosition;  // The current target position of the arm. runToPosition is called using this value at the very end of the loop, only once

    private boolean clawState = true;  // Whether the claw is closed (true) or open (false). Used as a toggle for user control of the claw

    private boolean shooterState;  // Whether or not the shooter is currently active

    private double clawEstimatedPosition;  // The actual estimated position of the claw (as opposed to the target position). Any inaccuracy here should be quickly corrected when the claw target is changed, and wouldn't cause major issues anyway (only bug I can think of is an impossible position check if FOOLPROOF_IMPOSSIBLE_POSITIONS is true). Managed by estimateServoPositions
    private double armEstimatedPosition;  // The actual estimated position (as opposed to the target position. This value is likely very close to the real position thanks to encoders). Managed by estimateServoPositions (yes, I know that this is a motor, but we're treating it like a servo here because we're using a set position rather than set power or velocity)

    private int currentSpeedCurve = DEFAULT_SPEED_CURVE;  // The speed curve and speed curve modes we're currently applying to user input, if enabled via USE_VARIABLE_SPEED_CURVES
    private int currentSpeedCurveMode = DEFAULT_SPEED_CURVE_MODE;

    // Drive motor objects
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Previous drive motor powers (for acceleration checking)
    private double frontLeftDrivePreviousPower = 0.0;
    private double frontRightDrivePreviousPower = 0.0;
    private double backLeftDrivePreviousPower = 0.0;
    private double backRightDrivePreviousPower = 0.0;

    // Drive motor powers
    private double frontLeftDrivePower;
    private double frontRightDrivePower;
    private double backLeftDrivePower;
    private double backRightDrivePower;

    // Intake motor object
    private DcMotor intakeDrive;

    // Intake motor power
    private double intakeDrivePower;

    // Wobble Goal manipulation motors and servos
    private Servo wobbleClaw;  // Wobble goal claw servo
    private Servo fingerServo;  // Ring finger servo
    private DcMotor wobbleArm;  // Wobble goal arm motor (used with encoders and runToPosition, so it acts like a servo)

    // Ring shooter motor
    private DcMotorEx ringShooter;

    // Ring Elevator motor
    private DcMotorEx ringElevator;

    private boolean slowMode;  // Whether or not we're currently going slower

    // Variables relating to wobble goal manipulation
    private boolean currentlyDeploying = false;  // Whether or not we're deploying the full wobble goal mechanism (claw, arm)
    private boolean currentlyUndeploying = false;  // Whether or not we're undeploying the full wobble goal mechanism (claw, arm)

    private long deploymentStartTime;  // The start time of the wobble goal deployment sequence to determine when to set servo positions (in milliseconds)
    private long undeploymentStartTime;  // The start time of the wobble goal undeployment sequence to determine when to set servo positions (in millseconds)

    private boolean clawUserControl = false;  // Whether or not the user has claimed control of the claw during the wobble goal deployment/undeployment. Only used if AUTO_PRIORITY is false. Reset when we're no longer deploying or undeploying or we switch from deployment/undeployment.

    // States for the servos/motors that can be controlled via toggles
    private boolean ringElevatorUp = false;  // Boolean representing whether or not the ring elevator is currently at (or running to) the up position
    private boolean armUp = false;  // Boolean representing whether or not the wobble goal arm is currently at (or running to) the up position

    // Gamepad 1 inputs JUST pressed
    private boolean gamepad1APressed = false;  // Whether or not the gamepad 1 a button was JUST pressed, handled by the handleInput function
    private boolean gamepad1BPressed = false;  // Whether or not the gamepad 1 b button was JUST pressed, handled by the handleInput function
    private boolean gamepad1XPressed = false;  // Whether or not the gamepad 1 x button was JUST pressed, handled by the handleInput function
    private boolean gamepad1YPressed = false;  // Whether or not the gamepad 1 y button was JUST pressed, handled by the handleInput function
    private boolean gamepad1LeftShoulderPressed = false;  // Whether or not the gamepad 1 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad1RightShoulderPressed = false;  // Whether or not the gamepad 1 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad1LeftStickPressed = false;  // Whether or not the gamepad 1 left stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad1RightStickPressed = false;  // Whether or not the gamepad 1 right stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadUpPressed = false;  // Whether or not the gamepad 1 dpad up button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadDownPressed = false;  // Whether or not the gamepad 1 dpad down button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadLeftPressed = false;  // Whether or not the gamepad 1 dpad left button was JUST pressed, handled by the handleInput function
    private boolean gamepad1DpadRightPressed = false;  // Whether or not the gamepad 1 dpad right button was JUST pressed, handled by the handleInput function

    // Gamepad 1 inputs being held
    private boolean gamepad1AHeld = false;  // Whether or not the gamepad 1 a button is being held, handled by the handleInput function
    private boolean gamepad1BHeld = false;  // Whether or not the gamepad 1 b button is being held, handled by the handleInput function
    private boolean gamepad1XHeld = false;  // Whether or not the gamepad 1 x button is being held, handled by the handleInput function
    private boolean gamepad1YHeld = false;  // Whether or not the gamepad 1 y button is being held, handled by the handleInput function
    private boolean gamepad1LeftShoulderHeld = false;  // Whether or not the gamepad 1 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad1RightShoulderHeld = false;  // Whether or not the gamepad 1 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad1LeftStickHeld = false;  // Whether or not the gamepad 1 left stick button is being held, handled by the handleInput function
    private boolean gamepad1RightStickHeld = false;  // Whether or not the gamepad 1 right stick button is being held, handled by the handleInput function
    private boolean gamepad1DpadUpHeld = false;  // Whether or not the gamepad 1 dpad up button is being held, handled by the handleInput function
    private boolean gamepad1DpadDownHeld = false;  // Whether or not the gamepad 1 dpad down button is being held, handled by the handleInput function
    private boolean gamepad1DpadLeftHeld = false;  // Whether or not the gamepad 1 dpad left button is being held, handled by the handleInput function
    private boolean gamepad1DpadRightHeld = false;  // Whether or not the gamepad 1 dpad right button is being held, handled by the handleInput function

    // Gamepad 2 inputs JUST pressed
    private boolean gamepad2APressed = false;  // Whether or not the gamepad 2 a button was JUST pressed, handled by the handleInput function
    private boolean gamepad2BPressed = false;  // Whether or not the gamepad 2 b button was JUST pressed, handled by the handleInput function
    private boolean gamepad2XPressed = false;  // Whether or not the gamepad 2 x button was JUST pressed, handled by the handleInput function
    private boolean gamepad2YPressed = false;  // Whether or not the gamepad 2 y button was JUST pressed, handled by the handleInput function
    private boolean gamepad2LeftShoulderPressed = false;  // Whether or not the gamepad 2 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad2RightShoulderPressed = false;  // Whether or not the gamepad 2 left shoulder button was JUST pressed, handled by the handleInput function
    private boolean gamepad2LeftStickPressed = false;  // Whether or not the gamepad 2 left stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad2RightStickPressed = false;  // Whether or not the gamepad 2 right stick button was JUST pressed, handled by the handleInput function
    private boolean gamepad2DpadUpPressed = false;  // Whether or not the gamepad 1 dpad up button is being held, handled by the handleInput function
    private boolean gamepad2DpadDownPressed = false;  // Whether or not the gamepad 2 dpad down button was JUST pressed, handled by the handleInput function
    private boolean gamepad2DpadLeftPressed = false;  // Whether or not the gamepad 2 dpad left button was JUST pressed, handled by the handleInput function
    private boolean gamepad2DpadRightPressed = false;  // Whether or not the gamepad 2 dpad right button was JUST pressed, handled by the handleInput function

    // Gamepad 2 inputs being held
    private boolean gamepad2AHeld = false;  // Whether or not the gamepad 2 a button is being held, handled by the handleInput function
    private boolean gamepad2BHeld = false;  // Whether or not the gamepad 2 b button is being held, handled by the handleInput function
    private boolean gamepad2XHeld = false;  // Whether or not the gamepad 2 x button is being held, handled by the handleInput function
    private boolean gamepad2YHeld = false;  // Whether or not the gamepad 2 y button is being held, handled by the handleInput function
    private boolean gamepad2LeftShoulderHeld = false;  // Whether or not the gamepad 2 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad2RightShoulderHeld = false;  // Whether or not the gamepad 2 left shoulder button is being held, handled by the handleInput function
    private boolean gamepad2LeftStickHeld = false;  // Whether or not the gamepad 2 left stick button is being held, handled by the handleInput function
    private boolean gamepad2RightStickHeld = false;  // Whether or not the gamepad 2 right stick button is being held, handled by the handleInput function
    private boolean gamepad2DpadUpHeld = false;  // Whether or not the gamepad 1 dpad up button is being held, handled by the handleInput function
    private boolean gamepad2DpadDownHeld = false;  // Whether or not the gamepad 2 dpad down button is being held, handled by the handleInput function
    private boolean gamepad2DpadLeftHeld = false;  // Whether or not the gamepad 2 dpad left button is being held, handled by the handleInput function
    private boolean gamepad2DpadRightHeld = false;  // Whether or not the gamepad 2 dpad right button is being held, handled by the handleInput function

    // Gamepad 1 axes
    private double gamepad1LeftStickX = 0.0;
    private double gamepad1LeftStickY = 0.0;
    private double gamepad1RightStickX = 0.0;
    private double gamepad1RightStickY = 0.0;
    private double gamepad1LeftTrigger = 0.0;
    private double gamepad1RightTrigger = 0.0;

    // Gamepad 2 axes
    private double gamepad2LeftStickX = 0.0;
    private double gamepad2LeftStickY = 0.0;
    private double gamepad2RightStickX = 0.0;
    private double gamepad2RightStickY = 0.0;
    private double gamepad2LeftTrigger = 0.0;
    private double gamepad2RightTrigger = 0.0;

    private long pModeStart = 0;
    private int Pmode = 0;

    private boolean autoDrive = false;  // Whether or not the robot is currently being driven automatically (through Roadrunner). Only applies if GAMEPAD_XY_TOGGLES_AUTO_DRIVE is true

    double vertical = 0.0;  // Corrections axes for the DriveSignals
    double horizontal = 0.0;
    double rotation = 0.0;  // Manged by PID using the below user input variable

    double verticalInput = 0.0;  // Movement axes for user input movement control
    double horizontalInput = 0.0;
    double rotationInput = 0.0;  // Input rotation from the user

    private double targetHeading;  // Offset to the robot heading, used as a target to maintain PID rotation
    private double targetHorizontal;
    private double targetVertical;

    private PIDFController headingController = new PIDFController(new PIDCoefficients(6.0, 1.0, 0.25), DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
    private PIDFController verticalController = new PIDFController(new PIDCoefficients(6.0, 1.0, 0.25), DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
    private PIDFController horizontalController = new PIDFController(new PIDCoefficients(6.0, 1.0, 0.25), DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);

    //private Pose2d currentPose = new Pose2d(-63, -52, Math.toRadians(90));
    private Pose2d currentPose = PoseStorage.currentPose;
    private final Pose2d initialPose = currentPose;
    private SampleMecanumDrive drive;

    private int autoPoseIndex = 0;  // Index of the pose to drive to automatically (if automatic driving is currently enabled), or -1 if no position should be driven to

    // The possible target positions for automatic driving
    public static Pose2d highGoalShootPose = PoseStorage.highGoalShootPose;  // Index 0
    public static Pose2d powerShotPose1 = PoseStorage.powerShot1Pose;  // Index 1
    public static Pose2d powerShotPose2 = PoseStorage.powerShot2Pose;  // Index 2
    public static Pose2d powerShotPose3 = PoseStorage.powerShot3Pose;  // Index 3
    public static Pose2d dpadControlPose = PoseStorage.highGoalShootPose;  // Should be overriden by the user when gamepad 1 Dpad buttons are pressed

    // Holds the positions of the actual goals to rotate towards (not poses to travel to!)
    public static List<Vector2d> goalPositions;

    // The trajectory we're currently following, if we're following a trajectory
    private Trajectory targetTrajectory;

    private int currentShooterTPS = highGoalTPS;  // The current TPS of the ring shooter, if active

    // The pose that we're currently targeting, extracted from the current index
    private Pose2d targetPose;

    private long fingerCooldown;  // Amount of time remaining until the finger will automatically go back inward to push another ring. Reset whenever the finger is used manually
    private boolean fingerAttemptingIntake;

    private boolean currentlyFollowingAutoTrajectory = false;  // Whether or not we're currently following an automatic trajectory. This should be set to false whenever we switch back to manual control

    private boolean firstRotate = true;  // Whether or not the robot has been rotated towards the current goal since it was last changed by anything other than the button that cycles just rotation

    private double continuousRotationEstimate;
    private double lastDrivePoseEstimate;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);  // The roadrunner representation of the robot, used for automatic rotation correction

        drive.setPoseEstimate(currentPose);

        telemetry.addLine("Initializing drive motors");  // Debug message

        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        telemetry.addLine("Initializing intake motor");  // Debug message

        // Initialize intake motor
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

        telemetry.addLine("Initializing ring dump servo");  // Debug message

        // Initialize ring dump servo
        //ringDump = hardwareMap.get(Servo.class, "ringDump");

        // Initialize flywheel motor
        ringShooter = hardwareMap.get(DcMotorEx.class, "shoot");

        telemetry.addLine("Initializing wobble goal manipulation motors");  // Debug message

        // Initialize Wobble Goal manipulation motors
        //wobbleLift = hardwareMap.get(DcMotor.class, "WGLift");
        //wobblePickup = hardwareMap.get(Servo.class, "WGPickup");
        wobbleClaw = hardwareMap.get(Servo.class, "WGClaw");
        wobbleArm = hardwareMap.get(DcMotor.class, "WGArm");
        fingerServo = hardwareMap.get(Servo.class, "ringFinger");

        // Initialize Ring Elevator motor
        ringElevator = hardwareMap.get(DcMotorEx.class, "ringElevator");
        ringElevator.setVelocityPIDFCoefficients(5, 3, 3, 0);

        telemetry.addLine("Initializing servo/motor positions/powers");  // Debug message

        initMotorPositions();

        lastDrivePoseEstimate = drive.getPoseEstimate().getHeading();
        continuousRotationEstimate = drive.getPoseEstimate().getHeading();

        targetHeading = initialPose.getHeading();
        targetHorizontal = initialPose.getX();
        targetVertical = initialPose.getY();

        time = System.currentTimeMillis();
        lastTime = time;  // Initialize the last tick time so that deltas can be properly calculated
        ringShooter.setVelocityPIDFCoefficients(150, 7, 10, 0);
    }

    @Override
    public void start() {
        goalPositions = new ArrayList<>();  // High goal + 3 power shots

        // FIXME: Just guessed at these
        goalPositions.add(new Vector2d(36, -24));  // High goal position (FIXME: Needs to be corrected for blue goal)
        goalPositions.add(new Vector2d(36, -6));  // Power shot 1 position
        goalPositions.add(new Vector2d(36, -12));  // Power shot 2 position
        goalPositions.add(new Vector2d(36, -18));  // Power shot 3 position

        goalPositions.add(goalPositions.get(0));  // Placeholder position for Dpad movement index 4
    }

    public double distanceToShooterVelocity(double distance) {
        return (25.0*distance)/14.0 + (9900.0/7.0);  // Guessed at this
    }

    public double absoluteMin(double a, double b, double c) {
        if (Math.abs(a) < Math.abs(b)) {
            // a < b; compare a to c now
            if (Math.abs(a) < Math.abs(c)) {  // a < b and a < c
                // a is smallest
                return a;
            }
            else {  // b > a > c
                // c is smallest
                return c;
            }
        }
        else if (Math.abs(b) < Math.abs(c)) {  // a > b; compare b to c now
            // b is smallest
            return b;
        }
        else {  // a > b > c
            // c is smallest
            return c;
        }
    }

    public double getRotationDelta(double lastRotation, double thisRotation) {
        double lastDegrees = Math.toDegrees(lastRotation);
        double thisDegrees = Math.toDegrees(thisRotation);

        double possibleDifference1 = (thisDegrees+360 - lastDegrees);
        double possibleDifference2 = (thisDegrees - (lastDegrees+360));
        double possibleDifference3 = (thisDegrees - lastDegrees);
        return Math.toRadians(-absoluteMin(possibleDifference1, possibleDifference2, possibleDifference3));  // TODO: By absolute value
    }

    @Override
    public void loop() {
        lastTime = time;  // Save this time as the LAST time for the NEXT tick
        time = System.currentTimeMillis();  // Get the new current time
        deltaTime = time - lastTime;  // Calculate the delta time from the last frame

        continuousRotationEstimate += getRotationDelta(drive.getPoseEstimate().getHeading(), lastDrivePoseEstimate);
        lastDrivePoseEstimate = drive.getPoseEstimate().getHeading();

        handleInput();  // Handle all basic user input and convert to more useful, unified forms. This only sets user input values; it does nothing else with them. That's the job of the manual control functions.
        pMode();

        if ((currentlyDeploying || currentlyUndeploying) && !AUTO_PRIORITY) {
            checkAutoInterrupts();  // If automatic input doesn't take priority and we're doing a task, make sure user input outside the deadzone takes back control by setting the *UserControl variables (interrupting that part of the auto process).
        } else {  // If we're not doing a task or automatic overrides this anyway, make sure the relevant variables go back to false so that automatic isn't paused forever
            if (clawUserControl) {
                clawUserControl = false;
            }
        }

        if (gamepad1DpadUpPressed) { RING_ELEVATOR_UP_POSITION += 5; } // added as a failsafe to allow manual lift position tweaking as needed
        else if (gamepad1DpadDownPressed) { RING_ELEVATOR_UP_POSITION -= 5; }

        autoServoControl();  // Run automatic servo control (wobble goal mechanism deployment or undeployment) on whatever motors are "free" from manual control if such deployment is enabled (which it also checks for through the user input variables). DO NOT move this function call into a conditional. Without this function, timekeeping for the deployment and undeployment would act really weird if any manual control interrupted it (it would act really weird in a number of circumstances for that matter. Don't do it.)

        if (USE_VARIABLE_SPEED_CURVES && BUTTONS_CYCLE_SPEED_CURVES) {
            inputAdjustVariableSpeedCurves();  // If we're using variable speed curves and they can be adjusted by user input, handle user input to adjust them if necessary
        }

        estimateServoPositions();  // Estimate the actual servo positions for impossible position checks. FIXME: This might belong lower down or further up, or maybe even run multiple times. Shouldn't affect anything yet though

        if (!(currentlyDeploying || currentlyUndeploying) || !AUTO_PRIORITY) {
            applyManualServoControls();  // Run manual control on the wobble goal-related servos if there aren't automatic tasks that conflict with them
        }

        getAutoPoseIndex();  // Use user input changes to determine what pose index we should be at, and whether we should be driving automatically or not

        if (ENABLE_AUTO_DRIVE && autoDrive) {  // If we're currently driving automatically
            checkAutoMovementInterrupts();  // Disable automatic driving if any manual movement is detected that should override it
        }

        if (!ENABLE_AUTO_DRIVE || !autoDrive) {  // If we're not automatically driving the robot, apply manual movement controls
            applyManualMovementControls();  // The wobble-goal tasks don't prevent any manual movement, which is applied separately in this function
        }
        else {  // If we're using automatic (Roadrunner powered) controls, apply those
            applyAutomaticMovementControls();  // Make any changes necessary to get to that previous selected pose
        }

        if (FOOLPROOF_IMPOSSIBLE_POSITIONS) {
            impossiblePositionCheck();  // If we're preventing impossible physical positions through software, check for such occurences and fix them in both manual, automatic, and combinations of the two (it doesn't matter at this point because this function handles the computed motor values outside any such contexts)
        }

        if (ACCELERATION_CAP != 0.0) {
            limitAcceleration();  // If we're limiting the acceleration of movement-related motors, check all motor powers against their previous values and correct for acceleation if needed
        }

        sanityCheckMotorValues();  // Do basic sanity checks on all motor and servo values (make sure they're within range) and clip them if they aren't
        applyMotorValues();  // Finally, apply the fully-processed, non-conflicting, acceleration-limited, safe motor values to the motors and servos themselves
    }

    private void initMotorPositions() {
        // Init routine for motors and servos (default positions)

        // Set drive motor directions
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize servo positions
        wobbleClaw.setPosition(CLAW_CLOSED_POSITION);
        fingerServo.setPosition(RING_FINGER_OUT_POSITION);

        // Use the arm's current position to find the (relative) up and down positions to set the arm's encoder to later
        armPosition = wobbleArm.getCurrentPosition();
        wobbleArm.setTargetPosition(armPosition);
        wobbleArm.setPower(1.0);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armEstimatedPosition = armPosition;
        ARM_UP_POSITION = armPosition - ARM_UP_POSITION_DELTA;  // Get the up position of the arm with respect to the current position of the arm at init time, which is assumed. FIXME: We might add the delta instead
        ARM_DOWN_POSITION = armPosition - ARM_DOWN_POSITION_DELTA;  // Get the down position of the arm with respect to the current position of the arm at init time, which is assumed. FIXME: We might add the delta insted

        // Set Ring Elevator motor...
        ringElevator.setTargetPosition(ringElevator.getCurrentPosition());
        //ringElevator.setPower(RING_ELEVATOR_VELOCITY);
        ringElevator.setPower(0.8);
        ringElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run mode
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2000;
    }

    private void inputAdjustVariableSpeedCurves() {
        if (LEFT_STICK_RESETS_SPEED_CURVE && gamepad1LeftStickPressed) {  // If the left stick resets the speed curve and was just pressed, reset the speed curve
            currentSpeedCurve = 0;
            currentSpeedCurveMode = 0;

            telemetry.addLine("Gamepad 1 left stick pressed: reset speed curve and speed curve mode");  // Debug message
        }

        if (BUTTONS_CYCLE_SPEED_CURVES) {  // Also an important change
            if (gamepad1APressed) {  // Cycle speed curve modes
                if (currentSpeedCurveMode < 2) {
                    currentSpeedCurveMode++;  // Cycle
                } else {
                    currentSpeedCurveMode = 0;  // Wrap back around to the beginning
                }

                telemetry.addData("Gamepad 1 A: cycled speed curve mode to ", currentSpeedCurveMode);  // Debug message
            }

            if (gamepad1BPressed) {  // Cycle speed curves
                if (currentSpeedCurve < 7) {
                    currentSpeedCurve++;  // Cycle
                } else {
                    currentSpeedCurve = 0;  // Wrap back around to the beginning
                }

                telemetry.addData("Gamepad 1 B: cycled speed curve to", currentSpeedCurve);  // Debug message
            }
        }
    }

    private void impossiblePositionCheck() {
        // TODO: This currently does nothing. It should check for any number of impossible mechanical positions reachable by manually or automatically set servo positions, especially those that would potentially result in damage to servos or components of the robot
    }

    private void sanityCheckMotorValues() {  // Make sure no motor or servo values are outside their limits
        frontLeftDrivePower = Math.max(-1, Math.min(1, frontLeftDrivePower));
        frontRightDrivePower = Math.max(-1, Math.min(1, frontRightDrivePower));
        backLeftDrivePower = Math.max(-1, Math.min(1, backLeftDrivePower));
        backRightDrivePower = Math.max(-1, Math.min(1, backRightDrivePower));

        intakeDrivePower = Math.max(-1, Math.min(1, intakeDrivePower));

        clawPosition = Math.max(0, Math.min(1, clawPosition));
        //armPosition = Math.max(0, Math.min(1, armPosition));
    }

    private void getAutoPoseIndex() {
        if (gamepad1APressed) {
            autoPoseIndex = 0;  // High goal
            currentlyFollowingAutoTrajectory = false;
            autoDrive = true;
        }
//        else if (gamepad1XPressed) {
//            autoPoseIndex = 1;  // Power shot 1
//            currentlyFollowingAutoTrajectory = false;
//            autoDrive = true;
//        }
//        else if (gamepad1YPressed) {
//            autoPoseIndex = 2;  // Power shot 2
//            currentlyFollowingAutoTrajectory = false;
//            autoDrive = true;
//        }
//        else if (gamepad1BPressed) {
//            autoPoseIndex = 3;  // Power shot 3
//            currentlyFollowingAutoTrajectory = false;
//            autoDrive = true;
//        }
        else if (gamepad1BPressed) {  // Cycles goals and rotates robot towards new goal (connected with autoPoseIndex). Should probably not be used while auto-driving, but you hopefully wouldn't anyway
            // Setting target heading is okay because it's an accumulative value and will act as if user input directly caused the rotation adjustment
            if (!firstRotate) {
                if (autoPoseIndex < 4 && autoPoseIndex >= 0) {  // We're in the goal range and it's safe to increment
                    autoPoseIndex++;
                }
                else {  // It's outside the goal range (or would be if we incremented); set it to the first goal (the high goal)
                    autoPoseIndex = 0;
                }
            }
            else {
                firstRotate = false;
                autoPoseIndex = 0;
            }

            //targetHeading = drive.getPoseEstimate().vec().angleBetween(goalPositions.get(autoPoseIndex));  // Might need to multiply by -1 or do some 90 degree offset or something. We'll see FIXME: Awaiting testing
            targetHeading = Math.atan((drive.getPoseEstimate().getY()) - goalPositions.get(autoPoseIndex).getY()) / (drive.getPoseEstimate().getX() - goalPositions.get(autoPoseIndex).getX());
        }
        else if (gamepad1XPressed) {  // Cycles power shot goals. Could probably rewrite this logic I did at 12:00 AM too
            if (autoPoseIndex < 3 && autoPoseIndex > 0) {  // We're in the power shot range and it's safe to increment
                autoPoseIndex++;
            }
            else {  // It's outside the power shot range; set it to the first power shot
                autoPoseIndex = 1;  // Power shot 1
            }

            currentlyFollowingAutoTrajectory = false;
            autoDrive = true;
        }
        else if (gamepad1YPressed) {
            if (autoPoseIndex < 3 && autoPoseIndex > 0) {  // We're in the power shot range and it's safe to decrement
                autoPoseIndex--;
            }
            else {  // It's outside the power shot range; set it to the last power shot
                autoPoseIndex = 3;  // Power shot 3
            }

            currentlyFollowingAutoTrajectory = false;
            autoDrive = true;
        }
        else if (gamepad1DpadLeftPressed) {
            goalPositions.set(4, goalPositions.get(autoPoseIndex));  // Make sure the goal position doesn't change?
            autoPoseIndex = 4;  // Dpad control
            currentlyFollowingAutoTrajectory = false;
            autoDrive = true;
            dpadControlPose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + 8, drive.getPoseEstimate().getHeading());
        }
        else if (gamepad1DpadRightPressed) {
            goalPositions.set(4, goalPositions.get(autoPoseIndex));
            autoPoseIndex = 4;  // Dpad control
            currentlyFollowingAutoTrajectory = false;
            autoDrive = true;
            dpadControlPose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 8, drive.getPoseEstimate().getHeading());
        }
        //else if (gamepad1DpadLeftPressed) {
        //    goalPositions.set(4, goalPositions.get(autoPoseIndex));
        //    autoPoseIndex = 4;  // Dpad control
        //    currentlyFollowingAutoTrajectory = false;
        //    autoDrive = true;
        //    dpadControlPose = new Pose2d(drive.getPoseEstimate().getX() - 8, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());
        //}
        //else if (gamepad1DpadRightPressed) {
        //    goalPositions.set(4, goalPositions.get(autoPoseIndex));
        //    autoPoseIndex = 4;  // Dpad control
        //    currentlyFollowingAutoTrajectory = false;
        //    autoDrive = true;
        //    dpadControlPose = new Pose2d(drive.getPoseEstimate().getX() + 8, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());
        //}
        else if (gamepad1RightStickPressed) {  // Cancel auto following if the right stick is pressed
            autoDrive = false;
        }

        if (RIGHT_SHOULDER_RESETS_CURRENT_TARGET && gamepad1RightShoulderPressed) {
            // Reset the current target pose to our current pose
            switch (autoPoseIndex) {
                case (0):
                    drive.setPoseEstimate(new Pose2d(highGoalShootPose.getX(), highGoalShootPose.getY(), drive.getPoseEstimate().getHeading()));  // FIXME: These might be off by one tick. This is probably the least important "bug" in the code at any decent tick rate
                    telemetry.addLine("Updated current pose to high goal shoot pose");
                    break;
                case (1):
                    drive.setPoseEstimate(new Pose2d(powerShotPose1.getX(), powerShotPose1.getY(), drive.getPoseEstimate().getHeading()));
                    telemetry.addLine("Updated current pose to power shot 1 pose");
                    break;
                case (2):
                    drive.setPoseEstimate(new Pose2d(powerShotPose2.getX(), powerShotPose2.getY(), drive.getPoseEstimate().getHeading()));
                    telemetry.addLine("Updated current pose to power shot 2 pose");
                    break;
                case (3):
                    drive.setPoseEstimate(new Pose2d(powerShotPose3.getX(), powerShotPose3.getY(), drive.getPoseEstimate().getHeading()));
                    telemetry.addLine("Updated current pose to power shot 3 pose");
                    break;
            }

            currentlyFollowingAutoTrajectory = false;  // If we were following this target, stop; now we've already reached it. This is probably redundant
        }
    }

    private void autoServoControl() {
        if (gamepad2LeftStickPressed) {  // Cancel auto controls if the driver's left stick was pressed
            currentlyDeploying = false;
            currentlyUndeploying = false;

            telemetry.addLine("Gamepad 2 left stick pressed: cancelled wobble goal deployment/undeployment (if active)");  // Debug message
        }

        if (gamepad2XPressed) {  // Deploy
            if (!currentlyDeploying) {
                currentlyDeploying = true;  // Start deploying if we aren't already
                deploymentStartTime = time;  // Set the deployment start time
                telemetry.addLine("Gamepad 2 X pressed: deploying wobble goal mechanism");  // Debug message
            }
            else {
                telemetry.addLine("Gamepad 2 X pressed, but not deploying: already deploying");  // Debug message
            }

            currentlyUndeploying = false;  // If we're undeploying, stop
        }

        if (gamepad2YPressed) {  // Undeploy
            if (!currentlyUndeploying) {
                currentlyUndeploying = true;  // Start undeploying if we aren't already
                undeploymentStartTime = time;
                telemetry.addLine("Gamepad 2 Y pressed: undeploying wobble goal mechanism");  // Debug message
            }
            else {
                telemetry.addLine("Gamepad 2 Y pressed, but not undeploying: already undeploying");  // Debug message
            }

            currentlyDeploying = false;  // If we're deploying, stop
        }

        // Rewrite these for the new Mark 2 sequence
        //if (currentlyDeploying) {  // Currently disabled to prevent undesired behavior (FIXME: Write the actual code)
       /* if (false) {
            long elapsedTime = time - deploymentStartTime;  // Get the elapsed time to keep track of which servos should be moving
            telemetry.addData("Deployment elapsed time: ", elapsedTime);
            boolean canContinue = true;  // Whether or not we should continue attempting auto servo control

            if (elapsedTime < 500 && (!clawUserControl || AUTO_PRIORITY)) {  // If the claw hasn't been closed fully and we have control of it
                clawPosition = CLAW_CLOSED_POSITION;  // Close it
                clawState = true;
            }
            else if (elapsedTime < 500 && (clawUserControl && !AUTO_PRIORITY)) {  // If the user is preventing us from moving the claw
                canContinue = CONTINUE_AUTO_WITH_OVERRIDEN_DEPENDENCIES;  // Only continue if we're supposed to in this case
            }

            if (elapsedTime > 1000) {  // The deployment has finished
                telemetry.addLine("Finished wobble goal deployment sequence");
                currentlyDeploying = false;
            }
        }*/

        // Rewrite this for Mark 2 as well
        //if (currentlyUndeploying) {  // Currently disabled to prevent undesired behavior (FIXME: Write the actual code)
       /* if (false) {
            long elapsedTime = time - undeploymentStartTime;  // Get the elapsed time to keep track of which servos should be moving
            telemetry.addData("Undeployment elapsed time: ", elapsedTime);
            boolean canContinue = true;  // Whether or not we should continue attempting auto servo control

            if (elapsedTime < 300 && (!clawUserControl || AUTO_PRIORITY) && canContinue) {  // If the claw hasn't been opened fully and we have control of it and we're supposed to continue
                clawPosition = CLAW_OPENED_POSITION;  // Open it
                clawState = false;
            }
            else if (elapsedTime < 300 && (clawUserControl && !AUTO_PRIORITY)) {  // If the user is preventing us from moving the claw
                canContinue = CONTINUE_AUTO_WITH_OVERRIDEN_DEPENDENCIES;
            }

            if (elapsedTime < 1300 && elapsedTime > 800 && (!clawUserControl || AUTO_PRIORITY) && canContinue) {  // If the claw hasn't been closed fully and we have control of it and we're supposed to continue
                clawPosition = CLAW_CLOSED_POSITION;  // Close it
                clawState = true;
            }

            if (elapsedTime > 1300) {  // The undeployment has finished
                telemetry.addLine("Finished wobble goal undeployment sequence");
                currentlyUndeploying = false;
            }
        }*/
    }

    private void applyAutomaticMovementControls() {
        // If a target position index is currently set and we're not within a certain threshold of that position and we're not currently following a trajectory for automatic driving, make a new trajectory and follow it asynchronously
        if (autoPoseIndex != -1) {
            if (autoPoseIndex >= 0 && autoPoseIndex < 5) {
                switch (autoPoseIndex) {
                    case (0):
                        targetPose = highGoalShootPose;
                        break;
                    case (1):
                        targetPose = powerShotPose1;
                        break;
                    case (2):
                        targetPose = powerShotPose2;
                        break;
                    case (3):
                        targetPose = powerShotPose3;
                        break;
                    case (4):
                        targetPose = dpadControlPose;
                }

                //double distance = Math.sqrt(Math.pow(drive.getPoseEstimate().getX(), 2) - Math.pow(targetPose.getX(), 2));  // This was dead wrong
                double distance = Math.sqrt(Math.pow(drive.getPoseEstimate().getX() - targetPose.getX(), 2) + Math.pow(drive.getPoseEstimate().getY() - targetPose.getY(), 2));  // This should fix it
                double angularDistance = Math.abs(drive.getPoseEstimate().getHeading() - targetPose.getHeading());

                boolean areWeThereYet = (distance <= AUTO_DISTANCE_THRESHOLD && angularDistance <= AUTO_ANGULAR_DISTANCE_THRESHOLD);

                if (!areWeThereYet && !currentlyFollowingAutoTrajectory) {  // We aren't following a trajectory, but need to be
                    targetTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading(targetPose).build();
                    drive.followTrajectoryAsync(targetTrajectory);  // This would be path continuity exception after the first iteration if this code were accessible while currentlyFollowingAutoTrajectory were true
                    currentlyFollowingAutoTrajectory = true;
                }
                else if (areWeThereYet) {  // We've reached the target pose; set currentlyFollowingAutoTrajectory to false to reflect this
                    currentlyFollowingAutoTrajectory = false;
                }
            }
            else {  // We have an invalid automatic position. We should probably throw an error here
                // Maybe throw error???
            }
        }
    }

    private void applyManualMovementControls() {
        verticalInput = 0.0;  // Save each movement axis we'll use in its own variable
        horizontalInput = 0.0;
        rotationInput = 0.0;

        double currentPowerFactor = gamepad1LeftShoulderHeld ? SLOW_MODE_POWER_FACTOR : 1.0;

        if (AXIS_MOVEMENT) {  // If we're using axis movement
            if (FULLAXIS_CONTROL) {  // We're using the fullaxis movement scheme
                if (USE_VARIABLE_SPEED_CURVES) {  // If we're using speed curves, apply the current one
                    verticalInput = easeNormalized((-gamepad1LeftStickY * FULLAXIS_LEFT_WEIGHT - gamepad1RightStickY * FULLAXIS_RIGHT_WEIGHT), currentSpeedCurve, currentSpeedCurveMode);
                    horizontalInput = easeNormalized((gamepad1LeftStickX * FULLAXIS_LEFT_WEIGHT + gamepad1RightStickX * FULLAXIS_RIGHT_WEIGHT), currentSpeedCurve, currentSpeedCurveMode);
                    rotationInput = easeNormalized(gamepad1RightTrigger * gamepad1RightTrigger - gamepad1LeftTrigger * gamepad1LeftTrigger, currentSpeedCurve, currentSpeedCurveMode);
                } else {  // Otherwise, just use a flat (linear) curve by directly applying the joystick values
                    verticalInput = (-gamepad1LeftStickY * FULLAXIS_LEFT_WEIGHT - gamepad1RightStickY * FULLAXIS_RIGHT_WEIGHT);
                    horizontalInput = (gamepad1LeftStickX * FULLAXIS_LEFT_WEIGHT + gamepad1RightStickX * FULLAXIS_RIGHT_WEIGHT);
                    rotationInput = (gamepad1RightTrigger - gamepad1LeftTrigger);
                }
            }
            else {
                if (USE_VARIABLE_SPEED_CURVES) {  // If we're using speed curves, apply the current one
                    verticalInput = easeNormalized(-gamepad1LeftStickY, currentSpeedCurve, currentSpeedCurveMode);
                    horizontalInput = easeNormalized(gamepad1LeftStickX, currentSpeedCurve, currentSpeedCurveMode);
                    rotationInput = easeNormalized(gamepad1RightStickX, currentSpeedCurve, currentSpeedCurveMode);
                } else {  // Otherwise, just use a flat (linear) curve by directly applying the joystick values
                    verticalInput = -gamepad1LeftStickY;
                    horizontalInput = gamepad1LeftStickX;
                    rotationInput = gamepad1RightStickX;
                }
            }
        } else {  // If we're using dpad movement
            if (gamepad1DpadUpHeld && !gamepad1DpadDownHeld) {  // Up
                verticalInput = 1.0;
            }
            if (gamepad1DpadDownHeld && !gamepad1DpadUpHeld) {  // Down
                verticalInput = -1.0;
            }

            if (gamepad1DpadLeftHeld && !gamepad1DpadRightHeld) {  // Left
                horizontalInput = -1.0;
            }

            if (gamepad1DpadRightHeld && !gamepad1DpadLeftHeld) {  // Right
                horizontalInput = 1.0;
            }

            rotationInput = gamepad1LeftTrigger - gamepad1RightTrigger;  // In Dpad mode, left and right triggers on gamepad 1 rotate
        }

        verticalInput *= currentPowerFactor;
        horizontalInput *= currentPowerFactor;
        rotationInput *= currentPowerFactor;

        targetVertical -= verticalInput / 10.0;
        targetHorizontal -= horizontalInput / 10.0;
        targetHeading -= rotationInput / 10.0;

        headingController.setTargetPosition(targetHeading);
        horizontalController.setTargetPosition(targetHorizontal);
        verticalController.setTargetPosition(targetVertical);

        if (!(autoDrive && currentlyFollowingAutoTrajectory) || !ENABLE_AUTO_DRIVE) {  // We're not auto driving
            rotation = headingController.update(continuousRotationEstimate);  // This should NEVER be called when it doesn't have accurate feedback. Otherwise P will accumulate until the autoDrive ends, then a burst of rotation will spin the bot out of control until it corrects itself
            horizontal = horizontalController.update(drive.getPoseEstimate().getX());
            vertical = verticalController.update(drive.getPoseEstimate().getY());
        }

        Vector2d directionalVector = new Vector2d(horizontal, vertical);
        directionalVector = directionalVector.rotated(-drive.getPoseEstimate().getHeading());  // Apply field-oriented heading

        horizontal = directionalVector.getX():
        vertical = directionalVector.getY();

        // Set drive motor power
        frontLeftDrivePower = vertical + horizontal + rotation;
        frontRightDrivePower = vertical - horizontal - rotation;
        backLeftDrivePower = vertical - horizontal + rotation;
        backRightDrivePower = vertical + horizontal - rotation;

        // Set intake speed
        intakeDrivePower = (gamepad2LeftTrigger - gamepad2RightTrigger) * INTAKE_MAX_POWER;
    }

    private void checkAutoInterrupts() {  // Checks to see if any manual input will interfere with an automatic task, if manual input takes priority (AUTO_PRIORITY is false). If so, it disables the automatic overrides using a flag for each servo involved
        if (gamepad2BPressed) {  // This adjusts the clawState, which means that a user should take control rather than auto fighting him
            clawUserControl = true;
        }
    }

    private void checkAutoMovementInterrupts() {  // Checks to see if any manual input will interfere with an automatic trajectory
        //if (Math.abs(gamepad1LeftStickX) + Math.abs(gamepad1LeftStickY) + Math.abs(gamepad1RightStickX) + Math.abs(gamepad1RightStickY) + Math.abs(gamepad1LeftTrigger) + Math.abs(gamepad1RightTrigger) >= GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE*6) {
        //    // One or more of the axes exceeds the deadzone (FIXME: I know that this could be computed in a more tedious, albeit possibly more accurate way by testing each axis individually. We'll see how it works and change it if necessary)
        //    autoDrive = false;  // We're overriding auto control
        //    currentlyFollowingAutoTrajectory = false;  // Yes, this line is important
        //}

        if (Math.abs(gamepad1LeftStickX) > GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE) {
            autoDrive = false;
            currentlyFollowingAutoTrajectory = false;
        }
        else if (Math.abs(gamepad1LeftStickY) > GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE) {
            autoDrive = false;
            currentlyFollowingAutoTrajectory = false;
        }
        else if (Math.abs(gamepad1RightStickX) > GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE) {
            autoDrive = false;
            currentlyFollowingAutoTrajectory = false;
        }
        else if (Math.abs(gamepad1RightStickY) > GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE) {
            autoDrive = false;
            currentlyFollowingAutoTrajectory = false;
        }
        else if (Math.abs(gamepad1LeftTrigger) > GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE) {
            autoDrive = false;
            currentlyFollowingAutoTrajectory = false;
        }
        else if (Math.abs(gamepad1RightTrigger) > GLOBAL_AUTO_MOVEMENT_OVERRIDE_DEADZONE) {
            autoDrive = false;
            currentlyFollowingAutoTrajectory = false;
        }
    }

    private void applyManualServoControls() {
        // FIXME: This may need to be rewritten

        // Claw state
        if (!NATURAL_CLAW_CONTROL) {
            // Claw state adjustment based on user input toggle
            if (gamepad2BPressed) {
                clawState = !clawState;
            }
        } else {
            // Claw state adjustment based on consistent user input (holding the button)
            if (INVERT_NATURAL_CLAW_CONTROL) {  // Holding the button opens the claw
                clawState = !gamepad2BHeld;
            } else {  // Holding the button closes the claw
                clawState = gamepad2BHeld;
            }
        }

        // Shooter state
//        shooterState = gamepad2LeftShoulderHeld || gamepad2RightShoulderHeld;  // The shooter is only on when the left shoulder on gamepad 2 is held
        shooterState = ringElevatorUp;  // The shooter turns on when the ring elevator is up, and turns off when it's down

        //if (gamepad2RightShoulderHeld) {
        //    currentShooterTPS = powerShotTPS;
        //}
//        else if (gamepad2LeftShoulderHeld) {
        //if (ringElevatorUp) {
            //currentShooterTPS = highGoalTPS;
        //}
        //else {
        //    //currentShooterTPS = 0;
        //    ringShooter.setPower(0.0);
        //}

        //if (shooterState) {
        //    currentShooterTPS = autoPoseIndex == 0 ? highGoalTPS : powerShotTPS;  // Get the desired shooter TPS from the selected auto target index
        //    ringShooter.setVelocity(currentShooterTPS);  // When the shooter is active, set the velocity to the target rate of the shooter for the high goal
        //}
        //else {
        //    ringShooter.setPower(0.0);  // If the shooter is inactive, zero its power
        //}

        if (AUTO_TPS_SELECT) {  // If we're automatically selecting TPS for the shooter
            currentShooterTPS = (int) distanceToShooterVelocity(drive.getPoseEstimate().vec().distTo(goalPositions.get(autoPoseIndex)));
        }
        else {  // Manual TPS control
            if (gamepad2LeftShoulderPressed) {
                currentShooterTPS += 5;
            }
            if (gamepad2RightShoulderPressed) {
                currentShooterTPS -= 5;
            }
        }

        if (shooterState) {
            ringShooter.setVelocity(currentShooterTPS);
        }
        else {
            ringShooter.setPower(0.0);
        }

        // Arm state toggle
        if (gamepad2YPressed) {  // FIXME: Is this in use somewhere else
            armUp = !armUp;
            armPosition = armUp ? ARM_DOWN_POSITION : ARM_UP_POSITION;
            //wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Arm position (fine tuning via manual control if needed)
        //if (Math.abs(gamepad2LeftStickY) > 0.1) {
        //    armPosition += gamepad2LeftStickY * deltaTime/1000.0 * 10;
        //}

        wobbleArm.setTargetPosition(armPosition);

        if (gamepad2XPressed) {  // Manual control of the finger; moves it inward when pressed
            fingerAttemptingIntake = true;
            fingerCooldown = 0;  // Reset finger cooldown on manual input
        }
        else if (gamepad2XHeld) {  // Attempt to cycle the finger inward and outward based on a cooldown and whether or not the button is continually held
            if (fingerCooldown <= 0) {  // If the cooldown has counted down to zero, it's time to (attempt to) intake the finger
                fingerAttemptingIntake = true;
            }
            else if (fingerCooldown <= 500-150) {  // If the cooldown has counted down to 300, but not zero, it's time to (attempt to) move the finger out again
                fingerAttemptingIntake = false;  // Reset the intake flag if we're outtaking
            }
        }
        else {  // The button is released; attempt to move the finger outward
            fingerAttemptingIntake = false;  // Reset the intake flag if the button is released
        }

        if (fingerCooldown > 0) {
            fingerCooldown -= deltaTime;  // Decrement the finger cooldown unless it's below or equal to 0
        }
        else {
            fingerCooldown = 500;  // At 150 milliseconds, the finger is moved out. At 0 (500 from now) it's moved in again
        }

        // Adjust finger state based on whether it's attempting to move inward and whether it's possible without damaging the ring elevator
        if (Math.abs(ringElevator.getCurrentPosition() - RING_ELEVATOR_DOWN_POSITION) <= Math.abs(RING_ELEVATOR_UP_POSITION - RING_ELEVATOR_DOWN_POSITION) / 80) {
            fingerPosition = fingerAttemptingIntake ? RING_FINGER_OUT_POSITION : RING_FINGER_IN_POSITION;  // Move the finger inward if it's currently attempting to and the elevator position allows it
        }
        else {
            fingerPosition = fingerAttemptingIntake ? RING_FINGER_IN_POSITION : RING_FINGER_OUT_POSITION;  // Invert finger goals when the bucket
        }

        if (!(currentlyDeploying || currentlyUndeploying) || clawUserControl) {  // clawUserControl will be set if we've changed this claw state, so this never locks us out of input unless auto has priority
            // Claw movement
            clawPosition = clawState ? CLAW_CLOSED_POSITION : CLAW_OPENED_POSITION;
        }

        // Ring Elevator movement
        if (gamepad2APressed) {
            ringElevatorUp = !ringElevatorUp;
            ringElevator.setTargetPosition(ringElevatorUp ? RING_ELEVATOR_UP_POSITION : RING_ELEVATOR_DOWN_POSITION);
        }
    }

    private void estimateServoPositions() {
        // Currently a dummy function. Should share timekeeping with the auto control function somehow and try to figure out where the servos really are. This might require VERY precise tweaking of automatic servo start/end times to work well and should probably stay as-is until we do that
        clawEstimatedPosition = clawPosition;
        armEstimatedPosition = wobbleArm.getCurrentPosition();
    }

    private void limitAcceleration() {
        // Limit acceleration of all robot movement-related motor values by checking the motor power variables against their previous values and the delta time from the last tick
        if (Math.abs(frontLeftDrivePreviousPower - frontLeftDrivePower) / deltaTime > ACCELERATION_CAP) {  // If the absolute difference between the previous motor power and this one over the delta time exceeds our acceleration cap, clip it to the acceleration cap and recalculate the motor power
            if (frontLeftDrivePower > frontLeftDrivePreviousPower) {  // Try to maintain direction relative to the previous power
                frontLeftDrivePower = frontLeftDrivePreviousPower + ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            } else {
                frontLeftDrivePower = frontLeftDrivePreviousPower - ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            }
        }

        if (Math.abs(frontRightDrivePreviousPower - frontRightDrivePower) / deltaTime > ACCELERATION_CAP) {  // If the absolute difference between the previous motor power and this one exceeds our acceleration cap, clip it to the acceleration cap and recalculate the motor power
            if (frontRightDrivePower >= frontRightDrivePreviousPower) {  // Try to maintain direction relative to the previous power
                frontRightDrivePower = frontRightDrivePreviousPower + ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            } else {
                frontRightDrivePower = frontRightDrivePreviousPower - ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            }
        }

        if (Math.abs(backLeftDrivePreviousPower - backLeftDrivePower) / deltaTime > ACCELERATION_CAP) {  // If the absolute difference between the previous motor power and this one exceeds our acceleration cap, clip it to the acceleration cap and recalculate the motor power
            if (backLeftDrivePower > backLeftDrivePreviousPower) {  // Try to maintain direction relative to the previous power
                backLeftDrivePower = backLeftDrivePreviousPower + ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            } else {
                backLeftDrivePower = backLeftDrivePreviousPower - ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
                backLeftDrivePower = backLeftDrivePreviousPower - ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            }
        }

        if (Math.abs(backRightDrivePreviousPower - backRightDrivePower) / deltaTime > ACCELERATION_CAP) {  // If the absolute difference between the previous motor power and this one exceeds our acceleration cap, clip it to the acceleration cap and recalculate the motor power
            if (backRightDrivePower > backRightDrivePreviousPower) {  // Try to maintain direction relative to the previous power
                backRightDrivePower = backRightDrivePreviousPower + ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            } else {
                backRightDrivePower = backRightDrivePreviousPower - ACCELERATION_CAP * deltaTime;  // Increment the motor power by the acceleration cap
            }
        }
    }

    private void applyMotorValues() {  // Assumes all sanity checks have already been applied to the variables it uses in setPosition
        if (MOVEMENT_ROTATION_CORRECTION || ENABLE_AUTO_DRIVE) {  // If we're using rotation correction or automatic driving is enabled, we'll have to use roadrunner's functions to set power and correct rotation
            //telemetry.addData("directionalVector angle: ", drive.getPoseEstimate().getHeading());
            //telemetry.update();

            if (!(autoDrive && currentlyFollowingAutoTrajectory) || !ENABLE_AUTO_DRIVE) {  // If we're NOT currently automatic driving
                //drive.setDrivePower(
                //        new Pose2d(
                //                directionalVector.getX(),
                //                directionalVector.getY(),
                //                -rotation
                //        )
                //);

                if (drive.mode != SampleMecanumDrive.Mode.IDLE) {
                    drive.mode = SampleMecanumDrive.Mode.IDLE;
                }

                drive.setDriveSignal(new DriveSignal(
                        new Pose2d(
                                horizontal * DriveConstants.MAX_VEL,  // Was 40.0
                                vertical * DriveConstants.MAX_VEL,
                                rotation)));
            }

            // Update all roadrunner stuff (odometry, etc.)
            // Do not that this method completely evades acceleration checks and the like, leaving all calculations to roadrunner
            drive.update();

            // Read pose
            currentPose = drive.getPoseEstimate();

            // Reset pose estimate rotation if needed
            if (LEFT_SHOULDER_RECALIBRATES_ROTATION && gamepad1LeftShoulderPressed) {  // Reset the current pose estimate rotation to the initial pose rotation
                currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), Math.toRadians(90));
                drive.setPoseEstimate(currentPose);
                targetHeading = drive.getPoseEstimate().getHeading();
                lastDrivePoseEstimate = targetHeading;
                continuousRotationEstimate = targetHeading;
                telemetry.addLine("Gamepad 1 left shoulder pressed: reset current pose rotation");
            }

            PoseStorage.currentPose = currentPose;  // Sync in case autonomous runs after this? Likely necessary but won't hurt anything

            telemetry.addData("Current position x:", currentPose.getX());
            telemetry.addData("Current position y: ", currentPose.getY());
            telemetry.addData("Current rotation input: ", rotationInput);
            telemetry.addData("Current rotation compensation: ", rotation);
            telemetry.addData("Current flywheel speed: ", currentShooterTPS);
            telemetry.addData("Current target heading: ", targetHeading);
            telemetry.addData("Current heading: ", currentPose.getHeading());
            telemetry.addData("Current continuous heading", continuousRotationEstimate);
            telemetry.addData("Currently following trajectory: ", currentlyFollowingAutoTrajectory);
            telemetry.addData("Currently auto-driving: ", autoDrive);
            telemetry.addData("Currently busy: ", drive.isBusy());
            telemetry.addData("Currently idle: ", drive.mode == SampleMecanumDrive.Mode.IDLE);
            telemetry.update();

            intakeDrive.setPower(intakeDrivePower);
        }
        else {  // Otherwise, we won't be using roadrunner at all
            // Apply all motor values
            frontLeftDrive.setPower(frontLeftDrivePower * MOVEMENT_FACTOR);
            frontRightDrive.setPower(frontRightDrivePower * MOVEMENT_FACTOR);
            backLeftDrive.setPower(backLeftDrivePower * MOVEMENT_FACTOR);
            backRightDrive.setPower(backRightDrivePower * MOVEMENT_FACTOR);
            intakeDrive.setPower(intakeDrivePower);

            // Update previous motor powers before the original motor values are changed on the next loop iteration
            frontLeftDrivePreviousPower = frontLeftDrivePower;
            frontRightDrivePreviousPower = frontRightDrivePower;
            backLeftDrivePreviousPower = backLeftDrivePower;
            backRightDrivePreviousPower = backRightDrivePower;
        }

        // Apply all servo values
        wobbleClaw.setPosition(clawPosition);
        fingerServo.setPosition(fingerPosition);
    }

    private void handleInput() {
        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;  // Whether or not the gamepad 1 a button was held
        boolean gamepad1BWasHeld = gamepad1BHeld;  // Whether or not the gamepad 1 b button was held
        boolean gamepad1XWasHeld = gamepad1XHeld;  // Whether or not the gamepad 1 x button was held
        boolean gamepad1YWasHeld = gamepad1YHeld;  // Whether or not the gamepad 1 y button was held
        boolean gamepad1LeftShoulderWasHeld = gamepad1LeftShoulderHeld;  // Whether or not the gamepad 1 left shoulder button was held
        boolean gamepad1RightShoulderWasHeld = gamepad1RightShoulderHeld;  // Whether or not the gamepad 1 left shoulder button was held
        boolean gamepad1LeftStickWasHeld = gamepad1LeftStickHeld;  // Whether or not the gamepad 1 left stick button was held
        boolean gamepad1RightStickWasHeld = gamepad1RightStickHeld;  // Whether or not the gamepad 1 right stick button was held
        boolean gamepad1DpadUpWasHeld = gamepad1DpadUpHeld;  // Whether or not the gamepad 1 dpad up button was held
        boolean gamepad1DpadDownWasHeld = gamepad1DpadDownHeld;  // Whether or not the gamepad 1 dpad down button was held
        boolean gamepad1DpadLeftWasHeld = gamepad1DpadLeftHeld;  // Whether or not the gamepad 1 dpad left button was held
        boolean gamepad1DpadRightWasHeld = gamepad1DpadRightHeld;  // Whether or not the gamepad 1 dpad right button was held

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1BHeld = gamepad1.b;
        gamepad1XHeld = gamepad1.x;
        gamepad1YHeld = gamepad1.y;
        gamepad1LeftShoulderHeld = gamepad1.left_bumper;
        gamepad1RightShoulderHeld = gamepad1.right_bumper;
        gamepad1LeftStickHeld = gamepad1.left_stick_button;
        gamepad1RightStickHeld = gamepad1.right_stick_button;
        gamepad1DpadUpHeld = gamepad1.dpad_up;
        gamepad1DpadDownHeld = gamepad1.dpad_down;
        gamepad1DpadLeftHeld = gamepad1.dpad_left;
        gamepad1DpadRightHeld = gamepad1.dpad_right;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;
        gamepad1XPressed = !gamepad1XWasHeld && gamepad1XHeld;
        gamepad1YPressed = !gamepad1YWasHeld && gamepad1YHeld;
        gamepad1LeftShoulderPressed = !gamepad1LeftShoulderWasHeld && gamepad1LeftShoulderHeld;
        gamepad1RightShoulderPressed = !gamepad1RightShoulderWasHeld && gamepad1RightShoulderHeld;
        gamepad1LeftStickPressed = !gamepad1LeftStickWasHeld && gamepad1LeftStickHeld;
        gamepad1RightStickPressed = !gamepad1RightStickWasHeld && gamepad1RightStickHeld;
        gamepad1DpadUpPressed = !gamepad1DpadUpWasHeld && gamepad1DpadUpHeld;
        gamepad1DpadDownPressed = !gamepad1DpadDownWasHeld && gamepad1DpadDownHeld;
        gamepad1DpadLeftPressed = !gamepad1DpadLeftWasHeld && gamepad1DpadLeftHeld;
        gamepad1DpadRightPressed = !gamepad1DpadRightWasHeld && gamepad1DpadRightHeld;

        // Cache previous gamepad 2 inputs
        boolean gamepad2AWasHeld = gamepad2AHeld;  // Whether or not the gamepad 2 a button was held
        boolean gamepad2BWasHeld = gamepad2BHeld;  // Whether or not the gamepad 2 b button was held
        boolean gamepad2XWasHeld = gamepad2XHeld;  // Whether or not the gamepad 2 x button was held
        boolean gamepad2YWasHeld = gamepad2YHeld;  // Whether or not the gamepad 2 y button was held
        boolean gamepad2LeftShoulderWasHeld = gamepad2LeftShoulderHeld;  // Whether or not the gamepad 2 left shoulder button was held
        boolean gamepad2RightShoulderWasHeld = gamepad2RightShoulderHeld;  // Whether or not the gamepad 2 left shoulder button was held
        boolean gamepad2LeftStickWasHeld = gamepad2LeftStickHeld;  // Whether or not the gamepad 2 left stick button was held
        boolean gamepad2RightStickWasHeld = gamepad2RightStickHeld;  // Whether or not the gamepad 2 right stick button was held
        boolean gamepad2DpadUpWasHeld = gamepad2DpadUpHeld;  // Whether or not the gamepad 2 dpad up button was held
        boolean gamepad2DpadDownWasHeld = gamepad2DpadDownHeld;  // Whether or not the gamepad 2 dpad down button was held
        boolean gamepad2DpadLeftWasHeld = gamepad2DpadLeftHeld;  // Whether or not the gamepad 2 dpad left button was held
        boolean gamepad2DpadRightWasHeld = gamepad2DpadRightHeld;  // Whether or not the gamepad 2 dpad right button was held

        // Get new values from the actual gamepad 2
        gamepad2AHeld = gamepad2.a;
        gamepad2BHeld = gamepad2.b;
        gamepad2XHeld = gamepad2.x;
        gamepad2YHeld = gamepad2.y;
        gamepad2LeftShoulderHeld = gamepad2.left_bumper;
        gamepad2RightShoulderHeld = gamepad2.right_bumper;
        gamepad2LeftStickHeld = gamepad2.left_stick_button;
        gamepad2RightStickHeld = gamepad2.right_stick_button;
        gamepad2DpadUpHeld = gamepad2.dpad_up;
        gamepad2DpadDownHeld = gamepad2.dpad_down;
        gamepad2DpadLeftHeld = gamepad2.dpad_left;
        gamepad2DpadRightHeld = gamepad2.dpad_right;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad2APressed = !gamepad2AWasHeld && gamepad2AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad2BPressed = !gamepad2BWasHeld && gamepad2BHeld;
        gamepad2XPressed = !gamepad2XWasHeld && gamepad2XHeld;
        gamepad2YPressed = !gamepad2YWasHeld && gamepad2YHeld;
        gamepad2LeftShoulderPressed = !gamepad2LeftShoulderWasHeld && gamepad2LeftShoulderHeld;
        gamepad2RightShoulderPressed = !gamepad2RightShoulderWasHeld && gamepad2RightShoulderHeld;
        gamepad2LeftStickPressed = !gamepad2LeftStickWasHeld && gamepad2LeftStickHeld;
        gamepad2RightStickPressed = !gamepad2RightStickWasHeld && gamepad2RightStickHeld;
        gamepad2DpadUpPressed = !gamepad2DpadUpWasHeld && gamepad2DpadUpHeld;
        gamepad2DpadDownPressed = !gamepad2DpadDownWasHeld && gamepad2DpadDownHeld;
        gamepad2DpadLeftPressed = !gamepad2DpadLeftWasHeld && gamepad2DpadLeftHeld;
        gamepad2DpadRightPressed = !gamepad2DpadRightWasHeld && gamepad2DpadRightHeld;

        // Gamepad 1 axes
        gamepad1LeftStickX = (Math.abs(gamepad1.left_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.left_stick_x : 0.0;  // Apply deadzone. Values below the deadzone are "snapped" to zero
        gamepad1LeftStickY = (Math.abs(gamepad1.left_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.left_stick_y : 0.0;
        gamepad1RightStickX = (Math.abs(gamepad1.right_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.right_stick_x : 0.0;
        gamepad1RightStickY = (Math.abs(gamepad1.right_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.right_stick_y : 0.0;
        gamepad1LeftTrigger = (Math.abs(gamepad1.left_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.left_trigger : 0.0;
        gamepad1RightTrigger = (Math.abs(gamepad1.right_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad1.right_trigger : 0.0;

        // Gamepad 2 axes
        gamepad2LeftStickX = (Math.abs(gamepad2.left_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.left_stick_x : 0.0;  // Apply deadzone. Values below the deadzone are "snapped" to zero
        gamepad2LeftStickY = (Math.abs(gamepad2.left_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.left_stick_y : 0.0;
        gamepad2RightStickX = (Math.abs(gamepad2.right_stick_x) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.right_stick_x : 0.0;
        gamepad2RightStickY = (Math.abs(gamepad2.right_stick_y) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.right_stick_y : 0.0;
        gamepad2LeftTrigger = (Math.abs(gamepad2.left_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.left_trigger : 0.0;
        gamepad2RightTrigger = (Math.abs(gamepad2.right_trigger) >= JOYSTICK_INPUT_THRESHOLD) ? gamepad2.right_trigger : 0.0;
    }

    private double easeIndex(double value, int easingIndex, int easingMode) {
        // Takes an index for an easing function and another integer for an easing mode and applies returns the applied easing. This function assumes that the passed value is within the range 0 to 1. For values from -1 to 1, use easeNormalized.
        // Returns NaN if the easing index or mode couldn't be found
        switch (easingMode) {
            case 0:  // Easing in
                switch (easingIndex) {
                    case 0:
                        return easeLinear(value);
                    case 1:
                        return easeInSine(value);
                    case 2:
                        return easeInQuad(value);
                    case 3:
                        return easeInCubic(value);
                    case 4:
                        return easeInQuart(value);
                    case 5:
                        return easeInQuint(value);
                    case 6:
                        return easeInExpo(value);
                    case 7:
                        return easeInCirc(value);
                }

                break;
            case 1:  // Easing out
                switch (easingIndex) {
                    case 0:
                        return easeLinear(value);
                    case 1:
                        return easeOutSine(value);
                    case 2:
                        return easeOutQuad(value);
                    case 3:
                        return easeOutCubic(value);
                    case 4:
                        return easeOutQuart(value);
                    case 5:
                        return easeOutQuint(value);
                    case 6:
                        return easeOutExpo(value);
                    case 7:
                        return easeOutCirc(value);
                }
                break;
            case 2:  // Easing in-out
                switch (easingIndex) {
                    case 0:
                        return easeLinear(value);
                    case 1:
                        return easeInOutSine(value);
                    case 2:
                        return easeInOutQuad(value);
                    case 3:
                        return easeInOutCubic(value);
                    case 4:
                        return easeInOutQuart(value);
                    case 5:
                        return easeInOutQuint(value);
                    case 6:
                        return easeInOutExpo(value);
                    case 7:
                        return easeInOutCirc(value);
                }
                break;
        }

        return Float.NaN;
    }

    private double easeNormalized(double value, int easingIndex, int easingMode) {
        // Takes a value from -1 to 1, an easing function, and an easing mode. Returns the normalized eased value from -1 to 1
        double denormalized = Math.abs(value);  // De-normalize the value by separating it from its sign
        int sign = value < 0 ? -1 : 1;  // Get the sign as an integer (1 or -1, to make the following code easier because we can just multiply)

        double eased = easeIndex(denormalized, easingIndex, easingMode);  // Run the requested easing function on the denormalized value
        double easedNormalized = eased * sign;  // Convert from the normalized value to the original value space by adding the sign again through multiplication

        return easedNormalized;  // Return the eased, normalized value in its original space with its new curve
    }

    // Easing functions
    private double easeLinear(double value) {
        return value;
    }

    private double easeInSine(double value) {
        return 1 - Math.cos((value * Math.PI) / 2);
    }

    private double easeOutSine(double value) {
        return Math.sin((value * Math.PI) / 2);
    }

    private double easeInOutSine(double value) {
        return -(Math.cos(Math.PI * value) - 1) / 2;
    }

    private double easeInQuad(double value) {
        return Math.pow(value, 2);
    }

    private double easeOutQuad(double value) {
        return 1 - Math.pow(1 - value, 2);
    }

    private double easeInOutQuad(double value) {
        return value < 0.5 ? 2 * Math.pow(value, 2) : 1 - Math.pow(-2 * value + 2, 2) / 2;
    }

    private double easeInCubic(double value) {
        return Math.pow(value, 3);
    }

    private double easeOutCubic(double value) {
        return 1 - Math.pow(1 - value, 3);
    }

    private double easeInOutCubic(double value) {
        return value < 0.5 ? 4 * Math.pow(value, 3) : 1 - Math.pow(-2 * value + 2, 3) / 2;
    }

    private double easeInQuart(double value) {
        return Math.pow(value, 4);
    }

    private double easeOutQuart(double value) {
        return 1 - Math.pow(1 - value, 4);
    }

    private double easeInOutQuart(double value) {
        return value < 0.5 ? 8 * Math.pow(value, 4) : 1 - Math.pow(-2 * value + 2, 4) / 2;
    }

    private double easeInQuint(double value) {
        return Math.pow(value, 5);
    }

    private double easeOutQuint(double value) {
        return 1 - Math.pow(1 - value, 5);
    }

    private double easeInOutQuint(double value) {
        return value < 0.5 ? 16 * Math.pow(value, 5) : 1 - Math.pow(-2 * value + 2, 5) / 2;
    }

    private double easeInExpo(double value) {
        return value == 0 ? 0 : Math.pow(2, 10 * value - 10);
    }

    private double easeOutExpo(double value) {
        return value == 1 ? 1 : 1 - Math.pow(2, -10 * value);
    }

    private double easeInOutExpo(double value) {
        return value == 0 ? 0 : value == 1 ? 1 : value < 0.5 ? Math.pow(2, 20 * value - 10) / 2 : (2 - Math.pow(2, -20 * value + 10)) / 2;
    }

    private double easeInCirc(double value) {
        return 1 - Math.sqrt(1 - Math.pow(value, 2));
    }

    private double easeOutCirc(double value) {
        return Math.sqrt(1 - Math.pow(value - 1, 2));
    }

    private double easeInOutCirc(double value) {
        return value < 0.5 ? (1 - Math.sqrt(1 - Math.pow(2 * value, 2))) / 2 : (Math.sqrt(1 - Math.pow(-2 * value + 2, 2)) + 1) / 2;
    }

    private void pMode() {
        int pmode1;
        int pmode2;
        int pmode;

        if (gamepad2DpadUpHeld) {
            pmode1 = 1;
        } else if (gamepad2DpadDownHeld) {
            pmode1 = 2;
        } else if (gamepad2DpadLeftHeld) {
            pmode1 = 3;
        } else if (gamepad2DpadRightHeld) {
            pmode1 = 4;
        } else {
            pmode1 = 0;
        }

        if (gamepad2RightStickY > 0.8) {
            pmode2 = 1;
        }
        else if (gamepad2RightStickY < -0.8) {
            pmode2 = 2;
        }
        else if (gamepad2RightStickX > 0.8) {
            pmode2 = 3;
        }
        else if (gamepad2RightStickX < -0.8) {
            pmode2 = 4;
        }
        else {
            pmode2 = 0;
        }

        pmode = (pmode1 << 8) + pmode2;

        if (pmode == Pmode && pmode != 0) {
            if (System.currentTimeMillis() - pModeStart > 2000) {
                Pmode = 0;

                if (pmode == 257) {
                    AXIS_MOVEMENT = gamepad2RightShoulderHeld;
                }
                else if (pmode == 258) {
                    AUTO_PRIORITY = gamepad2RightShoulderHeld;
                }
                else if (pmode == 260) {
                    USE_VARIABLE_SPEED_CURVES = gamepad2RightShoulderHeld;
                }
                else if (pmode == 513) {
                    FOOLPROOF_IMPOSSIBLE_POSITIONS = gamepad2RightShoulderHeld;
                }
                else if (pmode == 514) {
                    ACCELERATION_CAP += (gamepad2RightShoulderHeld) ? 1 : -1;
                    ACCELERATION_CAP = Math.max(0, Math.min(1, ACCELERATION_CAP));
                }
                else if (pmode == 516) {
                    NATURAL_CLAW_CONTROL = gamepad2RightShoulderHeld;
                }
                else if (pmode == 769) {
                    INVERT_NATURAL_CLAW_CONTROL = gamepad2RightShoulderHeld;
                }
                else if (pmode == 772) {
                    JOYSTICK_INPUT_THRESHOLD += (gamepad2RightShoulderHeld) ? 0.1 : -0.1;
                    JOYSTICK_INPUT_THRESHOLD = Math.max(0, Math.min(1, JOYSTICK_INPUT_THRESHOLD));
                }
                else if (pmode == 1025) {
                    MOVEMENT_FACTOR += (gamepad2RightShoulderHeld) ? 0.1 : -0.1;
                    MOVEMENT_FACTOR = Math.max(0, Math.min(1, MOVEMENT_FACTOR));
                }

                pModeStart = 0;
            }
        }
        else if (pmode != 0) {
            Pmode = pmode;
            pModeStart = System.currentTimeMillis();
        }
        else {
            Pmode = 0;
            pModeStart = 0;
        }
    }
}
