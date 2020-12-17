package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.DcMotorControl;

@TeleOp(name = "TeleOp0")

public class TeleOp0 extends OpMode {

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

    // Declare drive power
    private double frontLeftDrivePower;
    private double frontRightDrivePower;
    private double backLeftDrivePower;
    private double backRightDrivePower;

    // Declare chassis motion variables
    private double currentPower;
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
    private int wobbleGoalDeployState; // Integer from 0 to 3 representing the stage of deployment (lift, shoulder, claw, or done)
    private int wobbleGoalUndeployState; // Integer from 0 to 3 representing the sate of undeployment (claw, shoulder, lift, or done)

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
        wgPickup.setPosition(.32);
        wgShoulder.setPosition(0.0);                                                                              // Tune to Wobble Goal shoulder IN position
        wgClaw.setPosition(1.0);                                                                                  // Tune to Wobble Goal claw CLOSED position
    }

    @Override
    public void loop() {

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
        frontLeftDrive.setPower((vertical + horizontal + rotation) * powerLimiter);                               // Reverse in INIT if needed
        frontRightDrive.setPower((vertical - horizontal - rotation) * powerLimiter);                              // Reverse in INIT if needed
        backLeftDrive.setPower((vertical - horizontal + rotation) * powerLimiter);                                // Reverse in INIT if needed
        backRightDrive.setPower((vertical + horizontal - rotation) * powerLimiter);                               // Reverse in INIT if needed

// INTAKE CODE

        // Set intake power and mapping to controller input
        intakeDrive.setPower(DcMotorControl.motorIncrControl(gamepad2.right_trigger - gamepad2.left_trigger, intakeDrive.getPower()));      // Reverse in INIT if needed

// WOBBLE GOAL LIFT CODE

        // Restrict lift to only operate when Wobble Goal shoulder is rotated outside robot frame
        wgLift.setPower(wgShoulder.getPosition() < 1.0                                                                  // Change < 1 to restrict lift to only operate when shoulder is rotated out.
                        ? DcMotorControl.motorIncrControl(gamepad2.left_stick_y, wgLift.getPower())                      // Set Wobble Goal lift power and mapping to controller input  // Reverse in INIT if needed
                        : -Math.abs(DcMotorControl.motorIncrControl(gamepad2.left_stick_y, wgLift.getPower())));         // Only allow downward Wobble Goal lift movement if Wobble Goal shoulder is in chassis



// WOBBLE GOAL PICKUP CODE

        // Map Wobble Goal pickup to controller inputs
        wgPickup.setPosition(gamepad1.right_trigger >= 0.15 ? .7 : .32);

// WOBBLE GOAL SHOULDER CODE

        // Test for release of mapped button
        if (!gamepad2.a) { g2AReleased = true; }

        // Map Wobble Goal shoulder state switch to controller input
        if (gamepad2.a && g2AReleased) {
            g2AReleased = false;
            wgShoulderState = !wgShoulderState;
        }

        // Ternary operator to set Wobble Goal shoulder position
        wgShoulder.setPosition(wgShoulderState ? 0.0 : 1.0);                                                    // Tune to Wobble Goal shoulder IN / OUT position

// WOBBLE GOAL CLAW CODE

        // Test for release of mapped button
        if (!gamepad2.b) { g2BReleased = true; }

        // Map Wobble Goal claw state switch to controller input
        if (gamepad2.b && g2BReleased) {
            g2BReleased = false;
            wgClawState = !wgClawState;
        }

        // Ternary operator to set Wobble Goal claw position
        wgClaw.setPosition(wgClawState ? 0.0 : 1.0);                                                    // Tune to Wobble Goal claw OPEN / CLOSED position

        if (gamepad2.x) {
            // Set deploying
            wobbleGoalDeploying = true;
            wobbleGoalUndeploying = false;
        }

        if (gamepad2.y) {
            // Set undeploying
            wobbleGoalDeploying = false;
            wobbleGoalUndeploying = true;
        }

        if (wobbleGoalDeploying) {
            wobbleGoalDeploy();
            wobbleGoalDeployState = getWobbleGoalDeployedState();

            if (wobbleGoalDeployState == 3) { // We finished deploying; no need to keep running this block
                wobbleGoalDeploying = false;
            }
        }

        if (wobbleGoalUndeploying) {
            wobbleGoalUndeploy();
            wobbleGoalUndeployState = getWobbleGoalUndeployedState();

            if (wobbleGoalUndeployState == 3) { // We finished undeploying; no need to keep running this block
                wobbleGoalUndeploying = false;
            }
        }
    }

    private void wobbleGoalDeploy() { // FIXME: This is a dumb function name
        switch (wobbleGoalDeployState) {
            case 0:
                wgClaw.setPosition(WOBBLE_GOAL_DEPLOYED_CLAW_POSITION);
            break;

            case 1:
                wgShoulder.setPosition(WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION);
            break;

            case 2:
                wgLift.setPosition(WOBBLE_GOAL_DEPLOYED_LIFT_POSITION);
            break;
        }
    }

    private void wobbleGoalUndeploy() { // FIXME: Another bad name
        //wgLift.setPosition(WOBBLE_GOAL_UNDEPLOYED_LIFT_POSITION);
        //wgShoulder.setPosition(WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION);
        //wgClaw.setPosition(0.0); // Also guessing that this is closing

        switch (wobbleGoalDeployState) {
            case 0:
                wgLift.setPosition(WOBBLE_GOAL_UNDEPLOYED_LIFT_POSITION);
            break;

            case 1:
                wgShoulder.setPosition(WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION);
            break;

            case 2:
                wgClaw.setPosition(WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION);
            break;
        }
    }

    private int getWobbleGoalDeployedState() {
        //return (wgLift.getPosition() == WOBBLE_GOAL_DEPLOYED_LIFT_POSITION &&
        //wgShoulder.getPosition() == WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION &&
        //wgClaw.getPosition() == WOBBLE_GOAL_DEPLOYED_CLAW_POSITION);

        if (wbClaw.getPosition() == WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION) {
            if (wbShoulder.getPosition() == WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION) {
                if (wbLift.getPosition() == WOBBLE_GOAL_UNDEPLOYED_LIFT_POSITION) {
                    return 3;
                }

                return 2;
            }

            return 1;
        }

        return 0;
    }

    private int getWobbleGoalUndeployedState() {
        //return (wgLift.getPosition() == WOBBLE_GOAL_UNDEPLOYED_LIFT_POSITION &&
        //wgShoulder.getPosition() == WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION &&
        //wgClaw.getPosition() == WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION);

        if (wbLift.getPosition() == WOBBLE_GOAL_UNDEPLOYED_LIFT_POSITION) {
            if (wbShoulder.getPosition() == WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION) {
                if (wbClaw.getPosition() == WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION) {
                    return 3;
                }

                return 2;
            }

            return 1;
        }

        return 0;
    }

}
