package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOp99")

public class TeleOp99 extends OpMode {
    private static boolean AXIS_MOVEMENT = true;  // Whether or not rotation, horizontal, and vertical movement across the field should be controlled by joystick axes rather than dpad buttons

    private static double MOVEMENT_FACTOR = 1.0;  // Floating-point number from 0 to 1 multiplied by all movement motor powers directly before application. Use this to limit the maximum power for ALL movement-related motors evenly

    private static boolean AUTO_PRIORITY = false;  // Whether or not automatic motor and server controls take priority over manual ones instead of the other way around

    private static boolean CONTINUE_AUTO_WITH_OVERRIDEN_DEPENDENCIES = true;  // Whether the wobble goal deployment sequence should try to continue setting servo positions even if one of the dependency positions is manually set to something different TODO: This

    private static double JOYSTICK_INPUT_THRESHOLD = 0.10;  // The global threshold for all joystick axis inputs under which no input will be registered. Also referred to as a deadzone

    private static boolean GAMEPAD_XY_TOGGLES_AUTO_DRIVE = true;  // Whether or not the X and Y buttons on the driver gamepad should toggle Roadrunner-powered automatic driving. If this is true, X will enable and Y will disable automatic control

    private static boolean USE_VARIABLE_SPEED_CURVES = true;  // Whether or not custom curves for movement-related axis input should be used. If this is false, a linear curve will be used
    private static boolean BUTTONS_CYCLE_SPEED_CURVES = true;  // Only applies if using variable speed curves. If this is true, the driver's gamepad buttons (X and Y) will be able to cycle through custom speed curves. A toggles between in, out, and in-out easings and B selects a function (linear, sine, quad, cubic, quart, quint, expo, and circ in order of "curve sharpness")
    private static boolean LEFT_STICK_RESETS_SPEED_CURVE = true;  // Only applies if using variable speed curves. If this is true, the driver's gamepad joystick left stick will reset the speed curve to the default.
    private static int DEFAULT_SPEED_CURVE = 0;  // Only applies if using variable speed curves. The index of the speed curve to use by default (from 0-7 inclusive), and when reset. See above comments for speed curve list
    private static int DEFAULT_SPEED_CURVE_MODE = 0;  // Only applies if using variable speed curves. The index of the speed curve mode to use by default (from 0-2) inclusive. See above comments for mode list

    private static boolean FOOLPROOF_IMPOSSIBLE_POSITIONS = false;  // Whether or not servo position combinations which are impossible to reach physically will be prevented through software rather than servo gear-grinding, fire or accidental destruction of other parts. This only works to the point that such positions are predicted and tweaked accurately, and may be disabled under careful operation. TODO: This

    private static double ACCELERATION_CAP = 1.33333;  // The max allowed acceleration of any movement motor in units per second per second. This is included because max acceleration might tip the robot, but use with care as very low accelerations will make the robot sluggish. If the cap is reached, velocity will be incremented at this acceleration rather than the alternative. This value should be set to 1 divided by the number of seconds it should take for the motors to increment to maximum velocity. May be set above 1 for accelerations faster than 1 unit per second per second. 1.33333... (the default value) means it should take 0.75 seconds to go from 0 to full. Set to 0 to disable

    private static boolean MOVEMENT_ROTATION_CORRECTION = true;  // Whether or not we should attempt to adjust the robot's movement based on an accumulated rotation offset, which, if accurately maintained, would allow for rotating the robot without affecting movement from the driver's perspective. Disable this if steering seems to drift clockwise or counterclockwise after some amounts of rotation. DO NOTE THAT THIS OPTION EVADES ACCELERATION CHECKS. It does, however, respect all direct power limitations as well as any internal PID control from roadrunner (if any)

    private static boolean RESTRICT_LIFT_MOVEMENT = true;  // Whether or not the lift's movement should be disabled when the shoulder is in

    private static boolean NATURAL_CLAW_CONTROL = false;  // Whether or not the manual control for the claw should be a toggle. If this is true, holding the claw button will keep the claw closed, and releasing it will open the claw. If this is false, the claw button will toggle the state of the claw opened or closed
    private static boolean INVERT_NATURAL_CLAW_CONTROL = false;  // Only applies if NATURAL_CLAW_CONTROL is used. If this is true, holding the claw button with keep the claw open rather than closed. Likewise releasing the button will close the claw instead of opening it

    private static boolean NATURAL_SHOULDER_CONTROL = false;  // Whether or not the manual control for the shoulder should be a toggle. If this is true, holding the shoulder button will keep the shoulder out, and releasing it will swing the shoulder in. If this is false, the shoulder button will toggle the state of the shoulder in or out
    private static boolean INVERT_NATURAL_SHOULDER_CONTROL = false;  // Only applies if NATURAL_SHOULDER_CONTROL is used. If this is true, holding the shoulder button with keep the shoulder in rather than out. Likewise releasing the button will swing the shoulder out rather than in

    // FIXME: Are these the limits of the actual servo, or are they outside the existing range limit?
    // TODO: Sanity check should check using these if they're accurate
    private static double SHOULDER_MINIMUM = 0.0;  // The minimum position for the wobble goal shoulder servo
    private static double SHOULDER_MAXIMUM = 1.0;  // The maximum position for the wobble goal shoulder servo

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
    private static double RING_DUMP_COLLECT_POSITION = 0.5;  // The position of the ring dump when it's collecting

    private static double LIFT_POWER_MULTIPLIER = 0.35;  // The value multiplied to lift motor values to prevent snapping the line. Currently set to 25% of full power

    private static int RING_ELEVATOR_UP_POSITION;  // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION;  // The position of the Ring Elevator when it is in the DOWN state
    private static double RING_ELEVATOR_POWER = 0.3;  // The power for the motor to use when running to its target position

    private static double SLOW_MODE_POWER_FACTOR = 0.25;  // The amount multiplied to all motor values when in slow mode

    private static boolean FULLAXIS_CONTROL = true;  // Whether or not fullaxis mode is used. With this enabled, both thumb axes contribute equally (half power maximum on each joystick) to the final robot speed. In this mode, the gamepad 1 triggers are used for rotation, which also yields more movement and thus more overall control

    private static int PMODE = 0;  // PMODE, for problems

    private long time;  // The current time, used to measure delta time. Set at init and every loop iteration
    private long lastTime;  // The time of the LAST tick, used to measure delta time. Set at init and every loop iteration
    private long deltaTime;  // The delta time between ticks in milliseconds. Set at init and every loop iteration

    private double shoulderPosition;  // The current target position of the shoulder. setPosition is called using this value at the very end of the loop, only once
    private double clawPosition;  // The current target position of the claw. setPosition is called using this value at the very end of the loop, only once
    private double pickupPosition;  // The current target position of the pickup. setPosition is called using this value at the very end of the loop, only once
    private double liftPower;  // The current target power of the lift. setPower is called using this value at the very end of the loop, only once
    private double ringDumpPosition;  // The current target position of the ring dump. setPosition is called using this value at the very end of the loop, only once

    private boolean shoulderState;  // Whether the shoulder is out (true) or in (false). Used as a toggle for user control of the shoulder
    private boolean clawState;  // Whether the claw is closed (true) or open (false). Used as a toggle for user control of the claw

    private double shoulderEstimatedPosition;  // The actual estimated position of the shoulder (as opposed to the target position). Any inaccuracy here should be quickly corrected when the shoulder target is changed, and wouldn't cause major issues anyway (only bugs I can think of are the lift operation being temporarily affected if RESTRICT_LIFT_MOVEMENT is true or impossible positions being inaccurately checked if FOOLPROOF_IMPOSSIBLE_POSITIONS is true). Managed by estimateServoPositions
    private double clawEstimatedPosition;  // The actual estimated position of the claw (as opposed to the target position). Any inaccuracy here should be quickly corrected when the claw target is changed, and wouldn't cause major issues anyway (only bug I can think of is an impossible position check if FOOLPROOF_IMPOSSIBLE_POSITIONS is true). Managed by estimateServoPositions
    private double pickupEstimatedPosition;  // The actual estimated position of the pickup (as opposed to the target position). Any inaccuracy here should be quickly corrected when the lift target is changed, and wouldn't cause major issues anyway (only bug I can think of is an impossible position check if FOOLPROOF_IMPOSSIBLE_POSITIONS is true). Managed by estimateServoPositions.

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
    private DcMotor wobbleLift;
    private Servo wobblePickup;
    private Servo wobbleShoulder;
    private Servo wobbleClaw;

    // Ring manipulation servo
    private Servo ringDump;

    // Ring Elevator motor
    private DcMotor ringElevator;

    private boolean slowMode;  // Whether or not we're currently going slower

    // Variables relating to wobble goal manipulation
    private boolean currentlyDeploying = false;  // Whether or not we're deploying the full wobble goal mechanism (claw, pickup, and shoulder)
    private boolean currentlyUndeploying = false;  // Whether or not we're undeploying the full wobble goal mechanism (claw, pickup, and shoulder)

    private long deploymentStartTime;  // The start time of the wobble goal deployment sequence to determine when to set servo positions (in milliseconds)
    private long undeploymentStartTime;  // The start time of the wobble goal undeployment sequence to determine when to set servo positions (in millseconds)

    private boolean clawUserControl = false;  // Whether or not the user has claimed control of the claw during the wobble goal deployment/undeployment. Only used if AUTO_PRIORITY is false. Reset when we're no longer deploying or undeploying or we switch from deployment/undeployment.
    private boolean shoulderUserControl = false;  // Whether or not the user has claimed control of the shoulder during the wobble goal deployment/undeployment. Only used if AUTO_PRIORITY is false. Reset when we're no longer deploying or undeploying or we switch from deployment/undeployment
    private boolean pickupUserControl = false;  // Whether or not the user has claimed control of the pickup during the wobble goal deployment/undeployment. Only used if AUTO_PRIORITY is false. Reset when we're no longer deploying or undeploying or we switch from deployment/undeployment

    // Ring Elevator motor position
    private boolean ringElevatorUp = false;

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

    double vertical = 0.0;  // Movement axes for the manual robot movement control
    double horizontal = 0.0;
    double rotation = 0.0;

    private Pose2d currentPose = new Pose2d(-63, -52, Math.toRadians(90));  // TODO: SYNC THIS WITH AUTONOMOUS ASAP. PoseStorage should come in handy here
    private SampleMecanumDrive drive;

    private int autoPoseIndex = -1;  // Index of the pose to drive to automatically (if automatic driving is currently enabled), or -1 if no position should be driven to

    // The possible target positions for automatic driving
    public static Pose2d highGoalShootPose = new Pose2d();  // Index 0
    public static Pose2d powerShotPose1 = new Pose2d();  // Index 1
    public static Pose2d powerShotPose2 = new Pose2d();  // Index 2
    public static Pose2d powerShotPose3 = new Pose2d();  // Index 3

    // The trajectory we're currently following, if we're following a trajectory
    private Trajectory targetTrajectory;

    // The pose that we're currently targeting, extracted from the current index
    private Pose2d targetPose;

    private boolean currentlyFollowingAutoTrajectory = false;  // Whether or not we're currently following an automatic trajectory. This should be set to false whenever we switch back to manual control

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
        ringDump = hardwareMap.get(Servo.class, "ringDump");

        telemetry.addLine("Initializing wobble goal manipulation motors");  // Debug message

        // Initialize Wobble Goal manipulation motors
        wobbleLift = hardwareMap.get(DcMotor.class, "WGLift");
        wobblePickup = hardwareMap.get(Servo.class, "WGPickup");
        wobbleShoulder = hardwareMap.get(Servo.class, "WGShoulder");
        wobbleClaw = hardwareMap.get(Servo.class, "WGClaw");

        // Initialize Ring Elevator motor
        ringElevator = hardwareMap.get(DcMotor.class, "ringElevator");

        telemetry.addLine("Initializing servo/motor positions/powers");  // Debug message

        initMotorPositions();

        time = System.currentTimeMillis();
        lastTime = time;  // Initialize the last tick time so that deltas can be properly calculated
    }

    @Override
    public void start() {
        ringElevator.setPower(RING_ELEVATOR_POWER);
    }

    @Override
    public void loop() {
        lastTime = time;  // Save this time as the LAST time for the NEXT tick
        time = System.currentTimeMillis();  // Get the new current time
        deltaTime = time - lastTime;  // Calculate the delta time from the last frame

        handleInput();  // Handle all basic user input and convert to more useful, unified forms. This only sets user input values; it does nothing else with them. That's the job of the manual control functions.
        pMode();

        if ((currentlyDeploying || currentlyUndeploying) && !AUTO_PRIORITY) {
            checkAutoInterrupts();  // If automatic input doesn't take priority and we're doing a task, make sure user input outside the deadzone takes back control by setting the *UserControl variables (interrupting that part of the auto process).
        } else {  // If we're not doing a task or automatic overrides this anyway, make sure the relevant variables go back to false so that automatic isn't paused forever
            if (clawUserControl) {
                clawUserControl = false;
            }

            if (shoulderUserControl) {
                shoulderUserControl = false;
            }

            if (pickupUserControl) {
                pickupUserControl = false;
            }
        }

        autoServoControl();  // Run automatic servo control (wobble goal mechanism deployment or undeployment) on whatever motors are "free" from manual control if such deployment is enabled (which it also checks for through the user input variables). DO NOT move this function call into a conditional. Without this function, timekeeping for the deployment and undeployment would act really weird if any manual control interrupted it (it would act really weird in a number of circumstances for that matter. Don't do it.)

        if (USE_VARIABLE_SPEED_CURVES && BUTTONS_CYCLE_SPEED_CURVES) {
            inputAdjustVariableSpeedCurves();  // If we're using variable speed curves and they can be adjusted by user input, handle user input to adjust them if necessary
        }

        estimateServoPositions();  // Estimate the actual servo positions for impossible position checks or the lift operation check. FIXME: This might belong lower down or further up, or maybe even run multiple times. Shouldn't affect anything yet though

        if (!(currentlyDeploying || currentlyUndeploying) || !AUTO_PRIORITY) {
            applyManualServoControls();  // Run manual control on the wobble goal-related servos if there aren't automatic tasks that conflict with them
        }

        if (!GAMEPAD_XY_TOGGLES_AUTO_DRIVE || !autoDrive) {  // If we're not automatically driving the robot, apply manual movement controls
            applyManualMovementControls();  // The wobble-goal tasks don't prevent any manual movement, which is applied separately in this function
        }
        else {  // If we're using automatic (Roadrunner powered) controls, apply those
            getAutoPoseIndex();  // Use user input changes to determine what pose index we should be at
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
        wobblePickup.setPosition(PICKUP_UP_POSITION);
        wobbleShoulder.setPosition(SHOULDER_IN_POSITION);
        wobbleClaw.setPosition(CLAW_OPENED_POSITION);

        // Set Ring Elevator motor...
        ringElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run mode
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION - 2017;
    }

    private void inputAdjustVariableSpeedCurves() {
        if (LEFT_STICK_RESETS_SPEED_CURVE && gamepad1LeftStickPressed) {  // If the left stick resets the speed curve and was just pressed, reset the speed curve
            currentSpeedCurve = 0;
            currentSpeedCurveMode = 0;

            telemetry.addLine("Gamepad 1 left stick pressed: reset speed curve and speed curve mode");  // Debug message
        }

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

    private void impossiblePositionCheck() {
        // TODO: This currently does nothing. It should check for any number of impossible mechanical positions reachable by manually or automatically set servo positions, especially those that would potentially result in damage to servos or components of the robot
    }

    private void sanityCheckMotorValues() {  // Make sure no motor or servo values are outside their limits
        frontLeftDrivePower = Math.max(-1, Math.min(1, frontLeftDrivePower));
        frontRightDrivePower = Math.max(-1, Math.min(1, frontRightDrivePower));
        backLeftDrivePower = Math.max(-1, Math.min(1, backLeftDrivePower));
        backRightDrivePower = Math.max(-1, Math.min(1, backRightDrivePower));

        liftPower = Math.max(-1, Math.min(1, liftPower));
        intakeDrivePower = Math.max(-1, Math.min(1, intakeDrivePower));

        pickupPosition = Math.max(0, Math.min(1, pickupPosition));
        shoulderPosition = Math.max(0, Math.min(1, shoulderPosition));
        clawPosition = Math.max(0, Math.min(1, clawPosition));
        ringDumpPosition = Math.max(0, Math.min(1, ringDumpPosition));
    }

    private void getAutoPoseIndex() {
        // FIXME: Benjamin needs to program this. Tip: you can rely on the current pose and target pose in case one pose should automatically lead to another. Just wait until we're not currently following a trajectory before switching to the next one
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

        if (currentlyDeploying) {
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

            if (elapsedTime < 1000 && elapsedTime > 500 && (!shoulderUserControl || AUTO_PRIORITY) && canContinue) {  // If the shoulder hasn't been extended fully and we have control of it and we're supposed to continue
                shoulderPosition = SHOULDER_OUT_POSITION;  // Extend it
                shoulderState = false;
            }
            else if (elapsedTime < 1000 && elapsedTime > 500 && (shoulderUserControl && !AUTO_PRIORITY)) {  // If the user is preventing us from moving the shoulder
                canContinue = CONTINUE_AUTO_WITH_OVERRIDEN_DEPENDENCIES;  // Only continue if we're supposed to in this case
            }

            if (elapsedTime < 1000 && elapsedTime > 500 && (!pickupUserControl || AUTO_PRIORITY) && canContinue) {  // If the shoulder hasn't been extended fully and we have control of the pickup and we're supposed to continue
                pickupPosition = PICKUP_DOWN_POSITION;  // Lower the pickup (its movement is tied into the shoulder movement)
            }

            if (elapsedTime > 1000) {  // The deployment has finished
                telemetry.addLine("Finished wobble goal deployment sequence");
                currentlyDeploying = false;
            }
        }

        if (currentlyUndeploying) {
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

            if (elapsedTime < 800 && elapsedTime > 300 && (!shoulderUserControl || AUTO_PRIORITY)) {  // If the shoulder hasn't been retracted fully and we have control of it
                shoulderPosition = SHOULDER_IN_POSITION;  // Retract it
                shoulderState = true;
            }
            else if (elapsedTime < 800 && elapsedTime > 300 && (shoulderUserControl && !AUTO_PRIORITY)) {  // If the user is preventing us from moving the shoulder
                canContinue = CONTINUE_AUTO_WITH_OVERRIDEN_DEPENDENCIES;
            }

            if (elapsedTime < 800 && elapsedTime > 300 && (!pickupUserControl || AUTO_PRIORITY) && canContinue) {  // If the shoulder hasn't been retracted fully and we have control of the pickup and we're supposed to continue
                pickupPosition = PICKUP_UP_POSITION;  // Raise the pickup (its movement is tied into the shoulder movement)
            }
            else if (elapsedTime < 800 && elapsedTime > 300 && (!pickupUserControl && !AUTO_PRIORITY)) {
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
        }
    }

    private void applyAutomaticMovementControls() {
        // If a target position index is currently set and we're not within a certain threshold of that position and we're not currently following a trajectory for automatic driving, make a new trajectory and follow it asynchronously
        if (autoPoseIndex != -1) {
            if (autoPoseIndex >= 0 && autoPoseIndex < 4) {
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
                }

                if (currentPose != targetPose && !currentlyFollowingAutoTrajectory) {  // We aren't following a trajectory, but need to be
                    targetTrajectory = drive.trajectoryBuilder(PoseStorage.currentPose).lineToLinearHeading(targetPose).build();
                    drive.followTrajectoryAsync(targetTrajectory);  // This would be path continuity exception after the first iteration if this code were accessible while currentlyFollowingAutoTrajectory were true
                    currentlyFollowingAutoTrajectory = true;
                }
                else if (currentPose == targetPose) {  // We've reached the target pose; set currentlyFollowingAutoTrajectory to false to reflect this
                    currentlyFollowingAutoTrajectory = false;
                }
            }
            else {  // We have an invalid automatic position. We should probably throw an error here
                // Maybe throw error???
            }
        }
    }

    private void applyManualMovementControls() {
        vertical = 0.0;  // Save each movement axis we'll use in its own variable
        horizontal = 0.0;
        rotation = 0.0;

        double currentPowerFactor = gamepad1LeftShoulderHeld ? SLOW_MODE_POWER_FACTOR : 1.0;

        if (AXIS_MOVEMENT) {  // If we're using axis movement
            if (FULLAXIS_CONTROL) {  // We're using the fullaxis movement scheme
                if (USE_VARIABLE_SPEED_CURVES) {  // If we're using speed curves, apply the current one
                    vertical = easeNormalized((-gamepad1LeftStickY - gamepad1RightStickY) / 2.0, currentSpeedCurve, currentSpeedCurveMode);
                    horizontal = easeNormalized((gamepad1LeftStickX + gamepad1RightStickX) / 2.0, currentSpeedCurve, currentSpeedCurveMode);
                    rotation = easeNormalized(gamepad1LeftTrigger - gamepad1RightTrigger, currentSpeedCurve, currentSpeedCurveMode);
                } else {  // Otherwise, just use a flat (linear) curve by directly applying the joystick values
                    vertical = (-gamepad1LeftStickY - gamepad1RightStickY) / 2.0;
                    horizontal = (gamepad1LeftStickX + gamepad1RightStickX) / 2.0;
                    rotation = (gamepad1LeftTrigger - gamepad1RightTrigger);
                }
            }
            else {
                if (USE_VARIABLE_SPEED_CURVES) {  // If we're using speed curves, apply the current one
                    vertical = easeNormalized(-gamepad1LeftStickY, currentSpeedCurve, currentSpeedCurveMode);
                    horizontal = easeNormalized(gamepad1LeftStickX, currentSpeedCurve, currentSpeedCurveMode);
                    rotation = easeNormalized(gamepad1RightStickX, currentSpeedCurve, currentSpeedCurveMode);
                } else {  // Otherwise, just use a flat (linear) curve by directly applying the joystick values
                    vertical = -gamepad1LeftStickY;
                    horizontal = gamepad1LeftStickX;
                    rotation = gamepad1RightStickX;
                }
            }
        } else {  // If we're using dpad movement
            if (gamepad1DpadUpHeld && !gamepad1DpadDownHeld) {  // Up
                vertical = 1.0;
            }
            if (gamepad1DpadDownHeld && !gamepad1DpadUpHeld) {  // Down
                vertical = -1.0;
            }

            if (gamepad1DpadLeftHeld && !gamepad1DpadRightHeld) {  // Left
                horizontal = -1.0;
            }

            if (gamepad1DpadRightHeld && !gamepad1DpadLeftHeld) {  // Right
                horizontal = 1.0;
            }

            rotation = gamepad1LeftTrigger - gamepad1RightTrigger;  // In Dpad mode, left and right triggers on gamepad 1 rotate
        }

        vertical *= currentPowerFactor;
        horizontal *= currentPowerFactor;
        rotation *= currentPowerFactor;

        // Set drive motor power
        frontLeftDrivePower = vertical + horizontal + rotation;
        frontRightDrivePower = vertical - horizontal - rotation;
        backLeftDrivePower = vertical - horizontal + rotation;
        backRightDrivePower = vertical + horizontal - rotation;

        // Set intake speed
        intakeDrivePower = gamepad2RightTrigger - gamepad2LeftTrigger;
    }

    private void checkAutoInterrupts() {  // Checks to see if any manual input will interfere with an automatic task, if manual input takes priority (AUTO_PRIORITY is false). If so, it disables the automatic overrides using a flag for each servo involved
        if (gamepad2APressed) {  // This adjusts the shoulderState, which means that a user should take control rather than auto fighting him
            shoulderUserControl = true;
        }

        if (gamepad2BPressed) {  // This adjusts the clawState, which means that a user should take control rather than auto fighting him
            clawUserControl = true;
        }

        if (pickupPosition != (gamepad1RightTrigger >= JOYSTICK_INPUT_THRESHOLD ? PICKUP_DOWN_POSITION : PICKUP_UP_POSITION)) {  // If manual control would change the pickup position, allow it by letting the user take control instead of fighting him
            pickupUserControl = true;
        }
    }

    private void applyManualServoControls() {
        // Lift movement
        if (!RESTRICT_LIFT_MOVEMENT || (shoulderEstimatedPosition == SHOULDER_OUT_POSITION)) {  // Only allow the lift to move when the wobble goal shoulder is rotated out or RESTRICT_LIFT_MOVEMENT is false
            liftPower = LIFT_POWER_MULTIPLIER * -gamepad2LeftStickY;
        } else {  // If we've restricted the movement, make sure that the lift is either out of the way or getting there
            liftPower = 0.0;
        }

        if (!(currentlyDeploying || currentlyUndeploying) || shoulderUserControl) {  // shoulderUserControl will be set if we've changed this claw state, so this never locks us out of input unless auto has priority
            // Pickup movement, controlled by a gamepad trigger
            pickupPosition = gamepad1RightTrigger >= JOYSTICK_INPUT_THRESHOLD ? PICKUP_DOWN_POSITION : PICKUP_UP_POSITION;
        }

        // Shoulder state
        if (!NATURAL_SHOULDER_CONTROL) {
            // Shoulder state adjustment based on user input toggle
            if (gamepad2APressed) {
                shoulderState = !shoulderState;
            }
        } else {
            // Shoulder state adjustment based on consistent user input (holding the button)
            if (INVERT_NATURAL_SHOULDER_CONTROL) {  // Holding the button keeps the shoulder in
                shoulderState = gamepad2AHeld;
            } else {  // Holding the button keeps the shoulder out
                shoulderState = !gamepad2AHeld;
            }
        }

        if (!(currentlyDeploying || currentlyUndeploying) || shoulderUserControl) {  // shoulderUserControl will be set if we've changed this claw state, so this never locks us out of input unless auto has priority
            // Shoulder movement
            shoulderPosition = shoulderState ? SHOULDER_IN_POSITION : SHOULDER_OUT_POSITION;
        }

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

        if (!(currentlyDeploying || currentlyUndeploying) || clawUserControl) {  // clawUserControl will be set if we've changed this claw state, so this never locks us out of input unless auto has priority
            // Claw movement
            clawPosition = clawState ? CLAW_CLOSED_POSITION : CLAW_OPENED_POSITION;
        }

        // Ring dump movement
        ringDumpPosition = gamepad2LeftShoulderHeld ? RING_DUMP_DUMP_POSITION : RING_DUMP_COLLECT_POSITION;

        // Ring Elevator movement
        if (gamepad2APressed) {
            ringElevator.setTargetPosition(ringElevatorUp ? RING_ELEVATOR_DOWN_POSITION : RING_ELEVATOR_UP_POSITION);
            ringElevatorUp = !ringElevatorUp;
        }
    }

    private void estimateServoPositions() {
        // Currently a dummy function. Should share timekeeping with the auto control function somehow and try to figure out where the servos really are. This might require VERY precise tweaking of automatic servo start/end times to work well and should probably stay as-is until we do that
        pickupEstimatedPosition = pickupPosition;
        clawEstimatedPosition = clawPosition;
        shoulderEstimatedPosition = shoulderPosition;
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
        if (MOVEMENT_ROTATION_CORRECTION || GAMEPAD_XY_TOGGLES_AUTO_DRIVE) {  // If we're using rotation correction or automatic driving is enabled, we'll have to use roadrunner's functions to set power and correct rotation
            Vector2d directionalVector = new Vector2d(horizontal, vertical);
            directionalVector = directionalVector.rotated(-drive.getPoseEstimate().getHeading());
            //telemetry.addData("directionalVector angle: ", drive.getPoseEstimate().getHeading());
            //telemetry.update();

            drive.setDrivePower(
                    new Pose2d(
                            directionalVector.getX(),
                            directionalVector.getY(),
                            -rotation
                    )
            );

            // Update all roadrunner stuff (odometry, etc.)
            // Do not that this method completely evades acceleration checks and the like, leaving all calculations to roadrunner
            drive.update();

            // Read pose
            currentPose = drive.getPoseEstimate();  // We don't currently use this for anything, but roadrunner should automatically correct our rotation
            telemetry.addData("Current position x:", currentPose.getX());
            telemetry.addData("Current position y: ", currentPose.getY());
            telemetry.addData("Current heading: ", currentPose.getHeading());
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
        wobbleLift.setPower(liftPower);
        wobbleClaw.setPosition(clawPosition);
        wobblePickup.setPosition(pickupPosition);
        wobbleShoulder.setPosition(shoulderPosition);
        ringDump.setPosition(ringDumpPosition);
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
                else if (pmode == 259) {
                    CONTINUE_AUTO_WITH_OVERRIDEN_DEPENDENCIES = gamepad2RightShoulderHeld;
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
                else if (pmode == 515) {
                    RESTRICT_LIFT_MOVEMENT = gamepad2RightShoulderHeld;
                }
                else if (pmode == 516) {
                    NATURAL_CLAW_CONTROL = gamepad2RightShoulderHeld;
                }
                else if (pmode == 769) {
                    INVERT_NATURAL_CLAW_CONTROL = gamepad2RightShoulderHeld;
                }
                else if (pmode == 770) {
                    NATURAL_SHOULDER_CONTROL = gamepad2RightShoulderHeld;
                }
                else if (pmode == 771) {
                    INVERT_NATURAL_SHOULDER_CONTROL = gamepad2RightShoulderHeld;
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
