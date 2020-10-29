package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.Range;

public class DcMotorControl {

    static float stickClip = 0;
    static float power = 0;

    public static float motorControl(float analogValue) {
        stickClip = analogValue;
        stickClip = Range.clip(stickClip, -1, 1);

        if (stickClip > 0) { power += (float) .25; }
        else if (stickClip < 0) { power -= (float) .25; }
        else { power = 0; }

        if (stickClip > 0){
            if (power > stickClip) { power = stickClip; }
            if (power < .125) { power = (float) .125; }
            if (power > 1) { power = (float) 1; }
        }

        if (stickClip < 0){
            if (power < stickClip) { power = stickClip; }
            if (power > -.125) { power = (float) -.125; }
            if (power < -1) { power = (float) -1; }
        }

        return power;
    }

}
