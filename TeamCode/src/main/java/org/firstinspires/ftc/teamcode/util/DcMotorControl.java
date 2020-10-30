package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

public class DcMotorControl {

    private static final double MIN_POWER_THRESHOLD = 0.125;
    private static final double MIN_STICK_THRESHOLD = 0.125;
    private static final double POWER_INCR = 0.25;

    public static double motorIncrControl(double analogStick, double currentPower) {

        if (analogStick > MIN_STICK_THRESHOLD) {

            double powerClip = Range.clip(currentPower, MIN_POWER_THRESHOLD, 1.0);
            return (powerClip == currentPower + POWER_INCR) ? (currentPower + POWER_INCR) : powerClip;

        } else if (analogStick < -MIN_STICK_THRESHOLD) {

            double powerClip = Range.clip(currentPower, -1.0, -MIN_POWER_THRESHOLD);
            return (powerClip == currentPower - POWER_INCR) ? (currentPower - POWER_INCR) : powerClip;

        } else {

            return 0.0;

        }

    }

}
