package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.DcMotorControl;

@TeleOp(name = "TeleOp0")

public class TeleOp0 extends OpMode {

    // USER-DEFINED CONSTANTS
    private static final long WOBBLE_GOAL_DEPLOYED_CLAW_TIME = 500L;  // The amount of time it takes for the claw to be deployed
    private static final long WOBBLE_GOAL_DEPLOYED_SHOULDER_TIME = 1000L;  // The amount of time it takes for the shoulder to be deployed

    private static final double WOBBLE_GOAL_DEPLOYED_CLAW_POSITION = 0.31;  // The position that the claw is deployed to
    private static final double WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION = 0.27;  // The position that the shoulder is deployed to

    private static final double WOBBLE_GOAL_PICKUP_UP_POSITION = 0.32;
    private static final double WOBBLE_GOAL_PICKUP_DOWN_POSITION = 0.7;

    private static final long WOBBLE_GOAL_UNDEPLOYED_SHOULDER_TIME = 500L;
    private static final long WOBBLE_GOAL_UNDEPLOYED_CLAW_TIME = 1000L;

    private static final double WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION = 0.68;
    private static final double WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION = 0.89;

    private static final boolean DEPLOY_BLOCKS_INPUT = false;  // Whether or not deployment and undeployment should block basic input

    private static final float TRIGGER_ZERO_THRESHOLD = 0.15f;


    // Declare drive motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Declare intake motors
    private DcMotor intakeDrive;

    // Declare Wobble Goal manipulation motors
    private DcMotor wgLift;
    private Servo wgPickup;
    private Servo wgShoulder;
    private Servo wgClaw;

    // Declare chassis motion variables
    private double vertical;
    private double horizontal;
    private double rotation;

    // Declare Drive Power Limiting Variables
    private double powerLimiter;

    // Declare toggleable states
    private boolean g2AReleased;    // Used to determine if gamepad2.a has been released after pressing
    private boolean g2BReleased;    // Used to determine if gamepad2.a has been released after pressing
    private boolean wgShoulderState = false;  // IN
    private boolean wgClawState = true; // CLOSED

    private boolean wobbleGoalDeploying = false;
    private boolean wobbleGoalUndeploying = false;
    private long wobbleGoalDeployStartTime; // Float representing the starting time of deployment, used to determine the elapsed time
    private long wobbleGoalUndeployStartTime; // Float representing the starting time of undeployment, used to determine the elapsed time

    private boolean justDeployed = false;
    private boolean justUndeployed = false;

    @Override
    public void init() {

        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Initialize intake motors
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

        // Initialize Wobble Goal manipulation motors
        wgLift = hardwareMap.get(DcMotor.class, "WGLift");
        wgPickup = hardwareMap.get(Servo.class, "WGPickup");
        wgShoulder = hardwareMap.get(Servo.class, "WGShoulder");
        wgClaw = hardwareMap.get(Servo.class, "WGClaw");

        // Set drive motor directions
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servo positions
        wgPickup.setPosition(WOBBLE_GOAL_PICKUP_UP_POSITION);
        wgShoulder.setPosition(WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION);
        wgClaw.setPosition(WOBBLE_GOAL_DEPLOYED_CLAW_POSITION);

    }

    @Override
    public void loop() {
        if (!DEPLOY_BLOCKS_INPUT || !(wobbleGoalDeploying || wobbleGoalUndeploying)) { // If deployment doesn't block the input, or nothing is being deployed/undeployed, run the normal input-handling code
// DRIVE CODE

            // Toggleable drive speed limiter
            powerLimiter = gamepad1.left_bumper && !gamepad1.right_bumper ? 0.3     // Slow mode
                            : !gamepad1.left_bumper && gamepad1.right_bumper ? 1.0  // Fast mode
                            : 0.75;                                                 // Normal mode

            // Map vertical, horizontal, and rotational values to controller inputs
            vertical = DcMotorControl.motorIncrControl(-gamepad1.left_stick_y, vertical);
            horizontal = DcMotorControl.motorIncrControl(gamepad1.left_stick_x, horizontal);
            rotation = DcMotorControl.motorIncrControl(gamepad1.right_stick_x, rotation);

            // Set drive motor power
            frontLeftDrive.setPower((vertical + horizontal + rotation) * powerLimiter);                               // Reverse motor direction in INIT if needed
            frontRightDrive.setPower((vertical - horizontal - rotation) * powerLimiter);                              // Reverse motor direction in INIT if needed
            backLeftDrive.setPower((vertical - horizontal + rotation) * powerLimiter);                                // Reverse motor direction in INIT if needed
            backRightDrive.setPower((vertical + horizontal - rotation) * powerLimiter);                               // Reverse motor direction in INIT if needed

// INTAKE CODE

            // Set intake power and mapping to controller input
            intakeDrive.setPower(DcMotorControl.motorIncrControl(gamepad2.right_trigger - gamepad2.left_trigger, intakeDrive.getPower()));      // Reverse in INIT if needed

// WOBBLE GOAL LIFT CODE

            // Restrict lift to only operate when Wobble Goal shoulder is rotated outside robot frame
            wgLift.setPower(wgShoulder.getPosition() > WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION - .01                          // Change < 1 to restrict lift to only operate when shoulder is rotated out.
                            ? DcMotorControl.motorIncrControl(gamepad2.left_stick_y, wgLift.getPower())                      // Set Wobble Goal lift power and mapping to controller input  // Reverse in INIT if needed
                            : -Math.abs(DcMotorControl.motorIncrControl(gamepad2.left_stick_y, wgLift.getPower())));         // Only allow downward Wobble Goal lift movement if Wobble Goal shoulder is in chassis
// WOBBLE GOAL PICKUP CODE
            // Map Wobble Goal pickup to controller inputs
            if (!(wobbleGoalDeploying || wobbleGoalUndeploying)) {
                wgPickup.setPosition(gamepad1.right_trigger >= TRIGGER_ZERO_THRESHOLD ? WOBBLE_GOAL_PICKUP_DOWN_POSITION : WOBBLE_GOAL_PICKUP_UP_POSITION);
            }

// WOBBLE GOAL SHOULDER CODE

            // Test for release of mapped button
            if (!gamepad2.a) { g2AReleased = true; }

            // Map Wobble Goal shoulder state switch to controller input
            if (gamepad2.a && g2AReleased) {
                g2AReleased = false;
                wgShoulderState = !wgShoulderState;
            }

            // Ternary operator to set Wobble Goal shoulder position
            wgShoulder.setPosition(wgShoulderState ? WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION : WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION);  // Tune to Wobble Goal shoulder IN / OUT position

// WOBBLE GOAL CLAW CODE

            // Test for release of mapped button
            if (!gamepad2.b) { g2BReleased = true; }

            // Map Wobble Goal claw state switch to controller input
            if (gamepad2.b && g2BReleased) {
                g2BReleased = false;
                wgClawState = !wgClawState;
            }

            // Ternary operator to set Wobble Goal claw position
            wgClaw.setPosition(wgClawState ? WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION : WOBBLE_GOAL_DEPLOYED_CLAW_POSITION);  // Tune to Wobble Goal claw OPEN / CLOSED position
        } if (noUserInput() || DEPLOY_BLOCKS_INPUT) {  // If we're not using the normal user input code (deployment blocks user input AND we are deploying/undeploying something)

            if (justDeployed && wobbleGoalDeploying)
            {
              wobbleGoalDeployStartTime = System.currentTimeMillis();
              justDeployed = false;
            }
            if (justUndeployed && wobbleGoalUndeploying)
            {
              wobbleGoalUndeployStartTime = System.currentTimeMillis();
              justUndeployed = false;
            }

            if (wobbleGoalDeploying) {
                long currentTime = System.currentTimeMillis();
                wobbleGoalDeploy(currentTime);
            }

            if (wobbleGoalUndeploying) {
                long currentTime = System.currentTimeMillis();
                wobbleGoalUndeploy(currentTime);
            }
        }

// WOBBLE GOAL DEPLOYENT/UNDEPLOYMENT CODE

        if (gamepad2.x) {
            // Set deploying
            if (!wobbleGoalDeploying) {
                wobbleGoalDeploying = true;
                justDeployed = true;
                wobbleGoalDeployStartTime = System.currentTimeMillis();
            }

            wobbleGoalUndeploying = false;
        }

        if (gamepad2.y) {
            // Set undeploying
            if (!wobbleGoalUndeploying) {
                wobbleGoalUndeploying = true;
                justUndeployed = true;
                wobbleGoalUndeployStartTime = System.currentTimeMillis();
            }

            wobbleGoalDeploying = false;
        }

        if (gamepad2.right_stick_button) {
            // Failsafe to disable all deploying/undeploying, ensuring manual input is resumed
            wobbleGoalDeploying = false;
            wobbleGoalUndeploying = false;
        }
    }

    private void wobbleGoalDeploy(long currentTime) {
        long timeDelta = currentTime - wobbleGoalDeployStartTime;

        if (timeDelta < WOBBLE_GOAL_DEPLOYED_CLAW_TIME) {
            wgClaw.setPosition(WOBBLE_GOAL_DEPLOYED_CLAW_POSITION); // FIXME: Maybe this shouldn't be set every iteration
        }
        else if (timeDelta < WOBBLE_GOAL_DEPLOYED_SHOULDER_TIME) {
            wgShoulder.setPosition(WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION);
            wgPickup.setPosition(WOBBLE_GOAL_PICKUP_DOWN_POSITION);
            wobbleGoalDeploying = false;
        }

    }

    private void wobbleGoalUndeploy(long currentTime) {
        long timeDelta = currentTime - wobbleGoalUndeployStartTime;

        if (timeDelta < WOBBLE_GOAL_UNDEPLOYED_SHOULDER_TIME) {
            wgShoulder.setPosition(WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION);
            wgPickup.setPosition(WOBBLE_GOAL_PICKUP_UP_POSITION);
        }
        else if (timeDelta < WOBBLE_GOAL_UNDEPLOYED_CLAW_TIME) {
            wgClaw.setPosition(WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION);
            wobbleGoalUndeploying = false;
        }

    }

    /**
     * If there is currently no user input (you might need some variables to keep track of this but I don't know how it would work personally)
     */
    private boolean noUserInput() {
        // Check for all gamepad inputs that map to an action that would be overriden by wobble goal deploying and make sure they're within a reasonable deadzone. Return false otherwise.
        // Use the deadzone for axes. Buttons shouldn't need it.
        return !gamepad2.a && !gamepad2.b && (Math.abs(gamepad1.right_trigger) < TRIGGER_ZERO_THRESHOLD);
    }

}
