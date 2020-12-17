package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPositionFinder", group = "calibration")

public class ServoPositionFinder extends OpMode {

    private Servo servo0;

    private boolean rBumperReleased = false;
    private boolean lBumperReleased = false;

    @Override
    public void init() {

        servo0 = hardwareMap.get(Servo.class, "WGPickup");

        servo0.setPosition(.5);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper && lBumperReleased){
            lBumperReleased = false;
            servo0.setPosition(servo0.getPosition()-.02);
        }

        else if (!gamepad1.left_bumper) lBumperReleased = true;



        if (gamepad1.right_bumper && rBumperReleased){
            rBumperReleased = false;
            servo0.setPosition(servo0.getPosition()+.02);
        }

        else if (!gamepad1.right_bumper) rBumperReleased = true;

        telemetry.addData("Position = ", servo0.getPosition());
        telemetry.update();
    }
}
