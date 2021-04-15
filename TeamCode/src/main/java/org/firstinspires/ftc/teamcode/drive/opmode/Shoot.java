package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tnwutil.collections.Triplet;

@TeleOp(name = "Shoot")

public class Shoot extends OpMode {

    private DcMotor shoot;

    double shootPower = .5;
    int pulseTime = 1000; // milliseconds
    boolean pulsing = false;
    boolean firstIter = false;
    long iterStartTime;

    int shootCurrentPosition = 0;
    long currentTimeMillis = 0;

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

    double p = 0.0;
    double i = 0.0;
    double d = 0.0;

    @Override
    public void init() {
        telemetry.addLine("Initializing drive motors");  // Debug message

        // Initialize drive motors
        shoot = hardwareMap.get(DcMotor.class, "shoot");

    }

    void pulse(double power, int time, long iterStartTime) {
        shoot.setPower(power);
        if (currentTimeMillis >= iterStartTime + time) {
            shoot.setPower(0);
            pulsing = false;
        }
    }

    // To use the following function, declare the P, I, and D gains, the target RPM, last RPM,
    // previous integral, previous error, and delta time as fields in this class, providing all
    // of them on every call to this method. Sum the values in the returned Triplet of doubles
    // for the new motor power and telemetry log the rest, updating the appropriate reused fields
    // (previous <variable>)
    Triplet<Double, Double, Double> classicPID(double proportionalGain, double integralGain, double derivativeGain, int target, int lastValue, double previousIntegral, double previousError, long deltaTime) {
        double error = target - lastValue;

        double proportional = proportionalGain * error;
        double integral = previousIntegral + (integralGain * error * deltaTime);
        double derivative = derivativeGain * ((error - previousError) / deltaTime);

        return new Triplet<Double, Double, Double>(proportional, integral, derivative);
    }

    @Override
    public void loop() {
        currentTimeMillis = System.currentTimeMillis();

        if (gamepad1.a && !g1APressed) shootPower += powerIncr;
        else if (gamepad1.b && !g1BPressed) shootPower -= powerIncr;

        if (gamepad1.right_bumper && !g1RightBumperPressed) pulseTime += pulseTimeIncr;
        else if (gamepad1.left_bumper && !g1LeftBumperPressed) pulseTime -= pulseTimeIncr;

        if (gamepad1.x) {
            pulsing = true;
            iterStartTime = currentTimeMillis;
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

        if (currentTimeMillis >= lastIterTime + MILLIS_PER_ITER) {
            shootCurrentPosition = shoot.getCurrentPosition();
            rpm = (int) ((Math.abs(shootCurrentPosition - lastIterEncoder) / COUNTS_PER_REVOLUTION) / ((double) (currentTimeMillis - lastIterTime) / 60000));
            lastIterTime = currentTimeMillis;
            lastIterEncoder = shootCurrentPosition;
        }

        telemetry.addData("Encoder: ", lastIterEncoder);
        telemetry.addData("Speed: ", rpm);
        telemetry.addData("Power: ", shootPower);
        telemetry.addData("Pulse time: ", pulseTime);
        telemetry.update();

    }
}
