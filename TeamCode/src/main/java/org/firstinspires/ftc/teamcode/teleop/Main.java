package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.DcMotorControl;

public class Main extends OpMode {

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
    double frontLeftDrivePower;
    double frontRightDrivePower;
    double backLeftDrivePower;
    double backRightDrivePower;

    // Declare chassis motion variables
    double currentPower;
    double vertical;
    double horizontal;
    double rotation;

    // Declare Drive Power Limiting Variables
    double powerLimiter;

    // Declare toggleable states
    boolean g2AReleased;    // Used to determine if gamepad2.a has been released after pressing
    boolean g2BReleased;    // Used to determine if gamepad2.a has been released after pressing
    boolean wgShoulderState = false;  // IN
    boolean wgClawState = true; // CLOSED

    @Override
    public void init(){

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
        wgPickup.setPosition(1.0);                                                                                // Tune to Wobble Goal pickup UP position
        wgShoulder.setPosition(0.0);                                                                              // Tune to Wobble Goal shoulder IN position
        wgClaw.setPosition(1.0);                                                                                  // Tune to Wobble Goal claw CLOSED position
    }

    @Override
    public void loop(){

// DRIVE CODE

        // Toggleable drive speed limiter
        powerLimiter = gamepad1.left_bumper && !gamepad1.right_bumper ? 0.3     // Slow mode
                        : !gamepad1.left_bumper && gamepad1.right_bumper ? 1.0  // Fast mode
                        : 0.75;                                                 // Normal mode

        // Map vertical, horizontal, and rotational values to controller inputs
        vertical = DcMotorControl.motorIncrControl(-gamepad1.left_stick_y, 0.5);  // FIXME (multi-line) Input the actual current power of the respectful motor for each call of this method into currentPower
        horizontal = DcMotorControl.motorIncrControl(gamepad1.left_stick_x, 0.5);
        rotation = DcMotorControl.motorIncrControl(gamepad1.right_stick_x, 0.5);

        // Set drive motor power
        frontLeftDrive.setPower((vertical + horizontal + rotation) * powerLimiter);                               // Reverse in INIT if needed
        frontRightDrive.setPower((vertical - horizontal - rotation) * powerLimiter);                              // Reverse in INIT if needed
        backLeftDrive.setPower((vertical - horizontal + rotation) * powerLimiter);                                // Reverse in INIT if needed
        backRightDrive.setPower((vertical + horizontal - rotation) * powerLimiter);                               // Reverse in INIT if needed

// INTAKE CODE

        // Set intake power and mapping to controller input
        intakeDrive.setPower(DcMotorControl.motorIncrControl(gamepad2.right_trigger - gamepad2.left_trigger, 0.5));      // Reverse in INIT if needed

// WOBBLE GOAL LIFT CODE

        // Restrict lift to only operate when Wobble Goal shoulder is rotated outside robot frame
        wgLift.setPower(wgShoulder.getPosition() < 1.0                                                                  // Change < 1 to restrict lift to only operate when shoulder is rotated out.
                        ? DcMotorControl.motorIncrControl(gamepad2.left_stick_y, 0.5)                      // Set Wobble Goal lift power and mapping to controller input  // Reverse in INIT if needed
                        : -Math.abs(DcMotorControl.motorIncrControl(gamepad2.left_stick_y, 0.5)));         // Only allow downward Wobble Goal lift movement if Wobble Goal shoulder is in chassis



// WOBBLE GOAL PICKUP CODE

        // Map Wobble Goal pickup to controller inputs
        wgPickup.setPosition(gamepad1.right_trigger >= 0.15 ? 1.0 : 0.0);                                       // Tune to Wobble Goal pickup UP / DOWN position

// WOBBLE GOAL SHOULDER CODE

        // Test for release of mapped button
        if (!gamepad2.a) { g2AReleased = true; }

        // Map Wobble Goal shoulder state switch to controller input
        if (gamepad2.a && g2AReleased){
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

    }

}
