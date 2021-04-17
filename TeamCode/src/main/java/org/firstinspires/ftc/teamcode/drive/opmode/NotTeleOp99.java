package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "NotTeleOp99")

public class NotTeleOp99 extends OpMode {

    private DcMotor ringElevator;

    private boolean gamepad1AHeld = false;
    private boolean gamepad1APressed = false;

    private boolean ringElevatorUp = false;

    private static int RING_ELEVATOR_UP_POSITION; // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION; // The position of the Ring Elevator when it is in the DOWN state
    private static final double RING_ELEVATOR_POWER = 0.3; // The power for the motor to use when running to its target position

    @Override
    public void init() {
        ringElevator = hardwareMap.get(DcMotor.class, "ringElevator");

        // Set Ring Elevator motor...
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2017;
        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION);
        ringElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run mode
    }

    @Override
    public void start() {
        ringElevator.setPower(RING_ELEVATOR_POWER);
    }

    @Override
    public void loop() {
        handleInput();
        // Ring Elevator movement
        if (gamepad1APressed) {
            ringElevator.setTargetPosition(ringElevatorUp ? RING_ELEVATOR_DOWN_POSITION : RING_ELEVATOR_UP_POSITION);
            ringElevatorUp = !ringElevatorUp;
        }
    }

    private void handleInput() {
        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;  // Whether or not the gamepad 1 a button was held

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true

    }

}
