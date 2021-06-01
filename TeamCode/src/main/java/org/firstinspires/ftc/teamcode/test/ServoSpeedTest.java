package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Servo Speed")

public class ServoSpeedTest extends OpMode {

    private Servo servo1;
    final String SERVO_1_NAME = "servo1";

    int moveTime = 1000; // milliseconds
    double servo1MaxPosition = 1.0;
    long targetTime = 0; // Initialisation
    boolean atMax = false;

    boolean g1LeftBumperPressed = false;
    boolean g1RightBumperPressed = false;
    boolean g1APressed = false;
    boolean g1BPressed = false;

    final int moveTimeIncr = 50;
    final double positionIncr = .01;

    @Override
    public void init() {
        // Initialize drive motors
        servo1 = hardwareMap.get(Servo.class, SERVO_1_NAME);
    }

    @Override
    public void loop() {
        if (gamepad1.a && !g1APressed) moveTime += moveTimeIncr;
        else if (gamepad1.b && !g1BPressed) moveTime -= moveTimeIncr;

        if (gamepad1.right_bumper && !g1RightBumperPressed) servo1MaxPosition += positionIncr;
        else if (gamepad1.left_bumper && !g1LeftBumperPressed) servo1MaxPosition -= positionIncr;


        if (atMax && System.currentTimeMillis() >= targetTime) {
            servo1.setPosition(0);
            targetTime = System.currentTimeMillis() + moveTime;
            atMax = false;
        }
        else if (!atMax && System.currentTimeMillis() >= targetTime) {
            servo1.setPosition(servo1MaxPosition);
            targetTime = System.currentTimeMillis() + moveTime;
            atMax = true;
        }

        if (gamepad1.left_bumper) g1LeftBumperPressed = true;
        else g1LeftBumperPressed = false;
        if (gamepad1.right_bumper) g1RightBumperPressed = true;
        else g1RightBumperPressed = false;
        if (gamepad1.a) g1APressed = true;
        else g1APressed = false;
        if (gamepad1.b) g1BPressed = true;
        else g1BPressed = false;

        telemetry.addData("Range: ", servo1MaxPosition);
        telemetry.addData("Time: ", moveTime);
        telemetry.update();

    }
}
