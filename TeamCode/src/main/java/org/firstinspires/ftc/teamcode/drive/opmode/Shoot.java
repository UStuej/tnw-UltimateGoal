package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shoot")

public class Shoot extends OpMode {

    private DcMotor shoot;

    double shootPower = .5;
    int pulseTime = 1000; // milliseconds
    boolean pulsing = false;
    boolean firstIter = false;
    long iterStartTime;

    boolean g1LeftBumperPressed = false;
    boolean g1RightBumperPressed = false;
    boolean g1APressed = false;
    boolean g1BPressed = false;

    final double powerIncr = .05;
    final int pulseTimeIncr = 250;

    // RPM variables
    final int COUNTS_PER_REVOLUTION = 28; // REV HD Hex no gearbox
    final int MILLIS_PER_ITER = 100; // milliseconds
    int lastIterEncoder = 0;
    long lastIterTime = 0L;
    int rpm = 0;


    @Override
    public void init() {
        telemetry.addLine("Initializing drive motors");  // Debug message

        // Initialize drive motors
        shoot = hardwareMap.get(DcMotor.class, "shoot");

    }

    void pulse(double power, int time, long iterStartTime) {
        shoot.setPower(power);
        if (System.currentTimeMillis() >= iterStartTime + time) {
            shoot.setPower(0);
            pulsing = false;
        }
    }

    @Override
    public void loop() {

        if (gamepad1.a && !g1APressed) shootPower += powerIncr;
        else if (gamepad1.b && !g1BPressed) shootPower -= powerIncr;

        if (gamepad1.right_bumper && !g1RightBumperPressed) pulseTime += pulseTimeIncr;
        else if (gamepad1.left_bumper && !g1LeftBumperPressed) pulseTime -= pulseTimeIncr;

        if (gamepad1.x) {
            pulsing = true;
            iterStartTime = System.currentTimeMillis();
        }
        if (pulsing) {
            pulse(shootPower, pulseTime, iterStartTime);
        }

        if (gamepad1.left_bumper) g1LeftBumperPressed = true;
        else g1LeftBumperPressed = false;
        if (gamepad1.right_bumper) g1RightBumperPressed = true;
        else g1RightBumperPressed = false;
        if (gamepad1.a) g1APressed = true;
        else g1APressed = false;
        if (gamepad1.b) g1BPressed = true;
        else g1BPressed = false;

        if (System.currentTimeMillis() >= lastIterTime + MILLIS_PER_ITER) {
            rpm = (int) ((Math.abs(shoot.getCurrentPosition() - lastIterEncoder) / COUNTS_PER_REVOLUTION) / (System.currentTimeMillis() - lastIterTime));
            lastIterTime = System.currentTimeMillis();
            lastIterEncoder = shoot.getCurrentPosition();
        }

        telemetry.addData("Encoder: ", lastIterEncoder);
        telemetry.addData("Speed: ", rpm);
        telemetry.addData("Power: ", shootPower);
        telemetry.addData("Pulse time: ", pulseTime);
        telemetry.update();

    }
}
