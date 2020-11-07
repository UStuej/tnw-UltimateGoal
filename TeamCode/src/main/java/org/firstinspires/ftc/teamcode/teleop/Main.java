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
    private DcMotor WGLift;
    private Servo WGPickup;
    private Servo WGShoulder;
    private Servo WGClaw;

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

    //Declaring Drive Power Limiting Variables
    double powerLimiter;

    //Declaring toggleable states
    boolean g2AReleased;    // Used to determine if gamepad2.a has been released after pressing
    boolean g2BReleased;    // Used to determine if gamepad2.a has been released after pressing
    byte WGShoulderState = -1;  // IN
    byte WGClawState = 1; // CLOSED

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
        WGLift = hardwareMap.get(DcMotor.class, "WGLift");
        WGPickup = hardwareMap.get(Servo.class, "WGPickup");
        WGShoulder = hardwareMap.get(Servo.class, "WGShoulder");
        WGClaw = hardwareMap.get(Servo.class, "WGClaw");

        // Set drive motor directions
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Initialize servo positions
        WGPickup.setPosition(1);                                                                                // Tune to Wobble Goal pickup UP position
        WGShoulder.setPosition(0);                                                                              // Tune to Wobble Goal shoulder IN position
        WGClaw.setPosition(1);                                                                                  // Tune to Wobble Goal claw CLOSED position
    }

    @Override
    public void loop(){

// DRIVE CODE

        // Toggleable drive speed limiter
        if (gamepad1.left_bumper && !gamepad1.right_bumper) { powerLimiter = .3; }      // Slow mode
        else if (gamepad1.right_bumper && !gamepad1.left_bumper) { powerLimiter = 1; }  // Fast mode
        else { powerLimiter = .75; }                                                    // Normal mode

        // Mapping vertical, horizontal, and rotational values to controller inputs
        vertical = DcMotorControl.motorControl((-1*gamepad1.left_stick_y));
        horizontal = DcMotorControl.motorControl(gamepad1.left_stick_x);
        rotation = DcMotorControl.motorControl(gamepad1.right_stick_x);

        // Setting drive motor power
        frontLeftDrive.setPower((vertical + horizontal + rotation)*powerLimiter);                               // Reverse in INIT if needed
        frontRightDrive.setPower((vertical - horizontal - rotation)*powerLimiter);                              // Reverse in INIT if needed
        backLeftDrive.setPower((vertical - horizontal + rotation)*powerLimiter);                                // Reverse in INIT if needed
        backRightDrive.setPower((vertical + horizontal - rotation)*powerLimiter);                               // Reverse in INIT if needed

// INTAKE CODE

        // Setting intake power and mapping to controller input
        intakeDrive.setPower(DcMotorControl.motorControl((gamepad2.right_trigger-gamepad2.left_trigger)));      // Reverse in INIT if needed

// WOBBLE GOAL LIFT CODE

        // Restricting lift to only operate when Wobble Goal shoulder is rotated outside robot frame
        if (WGShoulder.getPosition() < 1){                                                                      // Change < 1 to restrict lift to only operate when shoulder is rotated out.
            // Setting Wobble Goal lift power and mapping to controller input
            WGLift.setPower(DcMotorControl.motorControl(gamepad2.left_stick_y));                                // Reverse in INIT if needed
        }
        // Only allows downward Wobble Goal lift movement if Wobble Goal shoulder is in chassis
        else { WGLift.setPower(Math.abs(DcMotorControl.motorControl(gamepad2.left_stick_y))*-1); }


// WOBBLE GOAL PICKUP CODE

        // Mapping Wobble Goal pickup to controller inputs
        if (gamepad1.right_trigger >= .15) { WGPickup.setPosition(1); }                                         // Tune to Wobble Goal pickup UP position
        else { WGPickup.setPosition(0); }                                                                       // Tune to Wobble Goal pickup DOWN position

// WOBBLE GOAL SHOULDER CODE

        // Testing for release of mapped button
        if (!gamepad2.a) { g2AReleased = true; }

        // Mapping Wobble Goal shoulder state switch to controller input
        if (gamepad2.a && g2AReleased){
            g2AReleased = false;
            WGShoulderState *= -1;  // Multiplying by -1 allows to swap states without using a series of "if" statements
            }

        // Switch case to set Wobble Goal shoulder position
        switch (WGShoulderState) {
            case -1:
                WGShoulder.setPosition(0);                                                                      // Tune to Wobble Goal shoulder IN position
                break;

            case 1:
                WGShoulder.setPosition(1);                                                                      // Tune to Wobble Goal shoulder OUT position
                break;
        }

// WOBBLE GOAL CLAW CODE

        // Testing for release of mapped button
        if (!gamepad2.b) { g2BReleased = true; }

        // Mapping Wobble Goal claw state switch to controller input
        if (gamepad2.b && g2BReleased) {
            g2BReleased = false;
            WGClawState *= -1;  // Multiplying by -1 allows to swap states without using a series of "if" statements
        }

        // Switch case to set Wobble Goal claw position
        switch (WGClawState) {
            case -1:
                WGClaw.setPosition(0);                                                                          // Tune to Wobble Goal claw OPEN position
                break;

            case 1:
                WGClaw.setPosition(1);                                                                          // Tune to Wobble Goal claw CLOSED position
                break;
        }

    }


    @Override
    public void stop(){

    }
}
