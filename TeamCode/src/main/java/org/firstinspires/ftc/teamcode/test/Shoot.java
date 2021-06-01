package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.tnwutil.collections.Triplet;

@Disabled
@Config
@TeleOp(name = "Shoot")

public class Shoot extends OpMode {

    private DcMotorEx shoot;

    FtcDashboard dashboard;

    public static int shootRps = 30;
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

    final int rpsIncr = 1;
    final int pulseTimeIncr = 250;

    // RPM variables
    final int COUNTS_PER_REVOLUTION = 28; // REV HD Hex no gearbox
    final int MILLIS_PER_ITER = 10; // milliseconds
    int lastIterEncoder = 0;
    long lastIterTime = 0L;
    public static int targetRps = 0;
    public static double rps = 0;

    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;

    PIDCoefficients PID = new PIDCoefficients(p, i, d);

    @Override
    public void init() {

        telemetry.addLine("Initializing drive motors");  // Debug message

        // Initialize drive motors
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

    }

    void pulse(double targetRps, int time, long iterStartTime) {
        shoot.setVelocity(targetRps * COUNTS_PER_REVOLUTION);
        if (currentTimeMillis >= iterStartTime + time) {
            shoot.setVelocity(0);
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
        shoot.setVelocityPIDFCoefficients(p, i, d, 0); // Ideal values as of 04/20/2020 - P:150 I:7 D:10

        if (gamepad1.a && !g1APressed) shootRps += rpsIncr;
        else if (gamepad1.b && !g1BPressed) shootRps -= rpsIncr;

        if (gamepad1.right_bumper && !g1RightBumperPressed) pulseTime += pulseTimeIncr;
        else if (gamepad1.left_bumper && !g1LeftBumperPressed) pulseTime -= pulseTimeIncr;

        if (gamepad1.x) {
            pulsing = true;
            iterStartTime = currentTimeMillis;
        }
        if (pulsing) {
            pulse(shootRps, pulseTime, iterStartTime);
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
            rps = (((double)(Math.abs(shootCurrentPosition - lastIterEncoder) / (double) COUNTS_PER_REVOLUTION) / ((double) (currentTimeMillis - lastIterTime) / 1000)));
            lastIterTime = currentTimeMillis;
            lastIterEncoder = shootCurrentPosition;
        }

        telemetry.addData("Encoder: ", lastIterEncoder);
        telemetry.addData("Speed: ", rps);
        telemetry.addData("Target: ", shootRps);
        telemetry.addData("Pulse time: ", pulseTime);
        telemetry.update();

    }
}
