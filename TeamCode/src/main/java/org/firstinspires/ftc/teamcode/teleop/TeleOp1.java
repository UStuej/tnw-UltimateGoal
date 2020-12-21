package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.DcMotorControl;

@TeleOp(name = "TeleOp1")

public class TeleOp1 extends OpMode {

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
    private float wobbleGoalDeployStartTime; // Float representing the starting time of deployment, used to determine the elapsed time
    private float wobbleGoalUndeployStartTime; // Float representing the starting time of undeployment, used to determine the elapsed time

    private float wobbleGoalDeployCurrentTime; // Float representing the current elapsed time of deployment for all stages, used to determine which servos to setPosition
    private float wobbleGoalUndeployCurrentTime; // Float representing the current elapsed time of undeployment for all stages, used to determine which servos to setPosition

    private const float WOBBLE_GOAL_DEPLOY_CLAW_TIME; // TODO: Define these
    private const float WOBBLE_GOAL_DEPLOY_SHOULDER_TIME;
    private const float WOBBLE_GOAL_DEPLOY_LIFT_TIME;
    private const float WOBBLE_GOAL_DEPLOY_FINISH_TIME;

    private const float WOBBLE_GOAL_DEPLOYED_CLAW_POSITION;
    private const float WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION;
    private const float WOBBLE_GOAL_DEPLOYED_LIFT_POSITION;

    private const float WOBBLE_GOAL_UNDEPLOY_LIFT_TIME;
    private const float WOBBLE_GOAL_UNDEPLOY_SHOULDER_TIME;
    private const float WOBBLE_GOAL_UNDEPLOY_CLAW_TIME;
    private const float WOBBLE_GOAL_UNDEPLOY_FINISH_TIME;

    private const float WOBBLE_GOAL_UNDEPLOYED_LIFT_POSITION;
    private const float WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION;
    private const float WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION;

    private float angleOffset = 0.0;
    private float angleOffsetFactor = 1.0;
    private float deltaTime = 0.0;

    private const boolean DEPLOY_BLOCKS_INPUT = false;  // Whether or not deployment and undeployment should block basic input

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
        wgShoulder.setPosition(0.0); // Tune to Wobble Goal shoulder IN position
        wgClaw.setPosition(1.0); // Tune to Wobble Goal claw CLOSED position
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
            //vertical = DcMotorControl.motorIncrControl(-gamepad1.left_stick_y, vertical);
            //horizontal = DcMotorControl.motorIncrControl(gamepad1.left_stick_x, horizontal);
            //rotation = DcMotorControl.motorIncrControl(gamepad1.right_stick_x, rotation);



            // Set drive motor power
            //frontLeftDrive.setPower((vertical + horizontal + rotation) * powerLimiter);                               // Reverse in INIT if needed
            //frontRightDrive.setPower((vertical - horizontal - rotation) * powerLimiter);                              // Reverse in INIT if needed
            //backLeftDrive.setPower((vertical - horizontal + rotation) * powerLimiter);                                // Reverse in INIT if needed
            //backRightDrive.setPower((vertical + horizontal - rotation) * powerLimiter);                               // Reverse in INIT if needed
            setDirection(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            deltaTime += System.currentTimeMillis - deltaTime;
            angleOffset += gamepad1.right_stick_y * angleOffsetFactor / deltaTime; // First offset will likely be incredibly small

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
            if (!gamepad2.a) {
                g2AReleased = true;
            }

            // Map Wobble Goal shoulder state switch to controller input
            if (gamepad2.a && g2AReleased) {
                g2AReleased = false;
                wgShoulderState = !wgShoulderState;
            }

            // Ternary operator to set Wobble Goal shoulder position
            wgShoulder.setPosition(wgShoulderState ? 0.0 : 1.0);                                                    // Tune to Wobble Goal shoulder IN / OUT position

// WOBBLE GOAL CLAW CODE

            // Test for release of mapped button
            if (!gamepad2.b) {
                g2BReleased = true;
            }

            // Map Wobble Goal claw state switch to controller input
            if (gamepad2.b && g2BReleased) {
                g2BReleased = false;
                wgClawState = !wgClawState;
            }

            // Ternary operator to set Wobble Goal claw position
            wgClaw.setPosition(wgClawState ? 0.0 : 1.0);                                                    // Tune to Wobble Goal claw OPEN / CLOSED position
        }

// WOBBLE GOAL DEPLOYENT/UNDEPLOYMENT CODE

        if (gamepad2.x) {
            // Set deploying
            if (!wobbleGoalDeploying) {
                wobbleGoalDeploying = true;
                wobbleGoalDeployStartTime = getTime();
            }

            wobbleGoalUndeploying = false;
        }

        if (gamepad2.y) {
            // Set undeploying
            if (!wobbleGoalUndeploying) {
                wobbleGoalUndeploying = true;
                wobbleGoalUndeployStartTime = getTime();
            }

            wobbleGoalDeploying = false;
        }

        if (gamepad2.right_stick_button) {
            // Failsafe to disable all deploying/undeploying, ensuring manual input is resumed
            wobbleGoalDeploying = false;
            wobbleGoalUndeploying = false;
        }

        if (wobbleGoalDeploying) {
            wobbleGoalDeploy();
            wobbleGoalDeployTime = getWobbleGoalDeployedTime();

            if (wobbleGoalDeployTime >= WOBBLE_GOAL_DEPLOY_FINISH_TIME) { // We finished deploying; no need to keep running this block
                wobbleGoalDeploying = false;
            }
        }

        if (wobbleGoalUndeploying) {
            wobbleGoalUndeploy();
            wobbleGoalUndeployTime = getWobbleGoalUndeployedTime();

            if (wobbleGoalUndeployTime >= WOBBLE_GOAL_UNDEPLOY_FINISH_TIME) { // We finished undeploying; no need to keep running this block
                wobbleGoalUndeploying = false;
            }
        }
    }

    private void wobbleGoalDeploy() {
        if (wobbleGoalDeployTime < WOBBLE_GOAL_DEPLOYED_CLAW_TIME) {
            wgClaw.setPosition(WOBBLE_GOAL_DEPLOYED_CLAW_POSITION); // FIXME: Maybe this shouldn't be set every iteration
        }
        else if (wobbleGoalDeployTime < WOBBLE_GOAL_DEPLOYED_SHOULDER_TIME) {
            wgShoulder.setPosition(WOBBLE_GOAL_DEPLOYED_SHOULDER_POSITION);
        }
        else if (wobbleGoalDeployTime < WOBBLE_GOAL_DEPLOYED_LIFT_TIME) {
            wgLift.setPosition(WOBBLE_GOAL_DEPLOYED_LIFT_POSITION);
        }

        }
    }

    private void wobbleGoalUndeploy() {
        if (wobbleGoalUndeployTime < WOBBLE_GOAL_UNDEPLOYED_LIFT_TIME) {
            wgLift.setPosition(WOBBLE_GOAL_UNDEPLOYED_LIFT_POSITION);
        }
        else if (wobbleGoalUndeployTime < WOBBLE_GOAL_UNDEPLOYED_SHOULDER_TIME) {
            wgShoulder.setPosition(WOBBLE_GOAL_UNDEPLOYED_SHOULDER_POSITION);
        }
        else if (wobbleGoalUndeployTime < WOBBLE_GOAL_UNDEPLOYED_CLAW_TIME) {
            wgClaw.setPosition(WOBBLE_GOAL_UNDEPLOYED_CLAW_POSITION);
        }

        }
    }

    private float getWobbleGoalDeployedTime() {
        return System.currentTimeMillis();
    }

    private float getWobbleGoalUndeployedTime() {
        return System.currentTimeMillis();
    }

    private void setDirection(float x, float y, float angle) {
        // Compute the initial magnitude of the joystick
        float joystickMagnitude = Math.sqrt(x**2 + y**2);

        // Compute the angle of the joystick vector
        float joystickAngle = Math.atan2(y, x) + angleOffset;

        // Compute the resulting motor values
        float frontLeftDrivePower = Math.sin(joystickAngle+(1/4*Math.PI)) * easeFunction(joystickMagnitude) + angle;
        float frontRightDrivePower = Math.sin(joystickAngle-(1/4*Math.PI)) * easeFunction(joystickMagnitude) + angle;
        float backLeftDrivePower = frontRightDrivePower;
        float backRightDrivePower = frontLeftDrivePower;

        float maxDrivePower = (Math.abs(frontLeftDrivePower) > Math.abs(frontRightDrivePower) ? Math.abs(frontLeftDrivePower) : Math.abs(frontRightDrivePower));

        frontLeftDrivePower /= maxDrivePower;
        frontRightDrivePower /= maxDrivePower;
        backLeftDrivePower /= maxDrivePower;
        backRightDrivePower /= maxDrivePower;

        // Set the motor values
        frontLeftDrive.setPower(frontLeftDrivePower);
        frontRightDrive.setPower(frontRightDrivePower);
        backLeftDrive.setPower(backLeftDrivePower);
        backRightDrive.setPower(backRightDrivePower);
    }

    private float easeInSine(float value) {
        return 1 - Math.cos((value * Math.PI) / 2);
    }

    private float easeOutSine(float value) {
        return Math.sin((value * Math.PI) / 2);
    }

    private float easeInOutSine(float value) {
        return -(Math.cos(Math.PI * x) - 1) / 2;
    }

    private float easeInQuad(float value) {
        return value**2;
    }

    private float easeOutQuad(float value) {
        1 - (1 - value)**2
    }

    private float easeInOutQuad(float value) {
        return value < 0.5 ? 2 * value**2 : 1 - Math.pow(-2 * value + 2, 2) / 2;
    }

    private float easeInCubic(float value) {
        return value**3;
    }

    private float easeOutCubic(float value) {
        return 1 - Math.pow(1 - value, 3);
    }

    private float easeInOutCubic(float value) {
        return value < 0.5 ? 4 * value**3 : 1 - Math.pow(-2 * value + 2, 3) / 2;
    }

    private float easeInQuart(float value) {
        return value**4;
    }

    private float easeOutQuart(float value) {
        return 1 - Math.pow(1 - value, 4);
    }

    private float easeInOutQuart(float value) {
        return value < 0.5 ? 8 * value**4 : 1 - Math.pow(-2 * value + 2, 4) / 2;
    }

    private float easeInQuint(float value) {
        return value**5;
    }

    private float easeOutQuint(float value) {
        return 1 - Math.pow(1 - value, 5);
    }

    private float easeInOutQuint(float value) {
        return value < 0.5 ? 16 * value**5 : 1 - Math.pow(-2 * value + 2, 5) / 2;
    }

    private float easeInExpo(float value) {
        return value == 0 ? 0 : Math.pow(2, 10 * value - 10);
    }

    private float easeOutExpo(float value) {
        return value == 1 ? 1 : 1 - Math.pow(2, -10 * value);
    }

    private float easeInOutExpo(float value) {
        return value == 0 ? 0 : value == 1 ? 1 : value < 0.5 ? Math.pow(2, 20 * value - 10) / 2 : (2 - Math.pow(2, -20 * value + 10)) / 2;
    }

    private float easeInCirc(float value) {
        return 1 - Math.sqrt(1 - Math.pow(value, 2));
    }

    private float easeOutCirc(float value) {
        return Math.sqrt(1 - Math.pow(value - 1, 2));
    }

    private float easeInOutCirc(float value) {
        return value < 0.5 ? (1 - Math.sqrt(1 - Math.pow(2 * value, 2))) / 2 : (Math.sqrt(1 - Math.pow(-2 * value + 2, 2)) + 1) / 2;
    }

    private float easeFunction(float value) {
        return value;  // Placeholder for any easing function we might use (if any)
    }

}
