package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

public class DcMotorControl {

    private static final double MIN_POWER_THRESHOLD = 0.35;
    private static final double MIN_STICK_THRESHOLD = 0.125;
    private static final double POWER_INCR = 0.25;

    public static double motorIncrControl(double analogStick, double currentPower) {

        return (analogStick > MIN_STICK_THRESHOLD)
                ? Range.clip(currentPower + POWER_INCR, MIN_POWER_THRESHOLD, 1.0)
                : ((analogStick < -MIN_STICK_THRESHOLD)
                ? Range.clip(currentPower - POWER_INCR, -1.0, -MIN_POWER_THRESHOLD)
                : 0.0);

    }

}
