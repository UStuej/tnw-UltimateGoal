package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tnwutil.collections.Triplet;

@TeleOp(name = "Motor Test")

public class MotorTest extends OpMode {

    private DcMotor motor1;
    private final String MOTOR1_NAME = "shoot";
    int targetPosition = 0;
    int incrAmount = 5;
    int position = 0;

    boolean g1APressed = false;
    boolean g1BPressed = false;



    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, MOTOR1_NAME);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setTargetPosition(0);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop() {

        if (gamepad1.a && !g1APressed) targetPosition += incrAmount;
        else if (gamepad1.b && !g1BPressed) targetPosition -= incrAmount;

        if (gamepad1.b) motor1.setPower(.3);
        else motor1.setPower(0);

        motor1.setTargetPosition(targetPosition);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        position = motor1.getCurrentPosition();

        telemetry.addData("Current Position: ", position);
        telemetry.addData("Target Position: ", targetPosition);

        if (gamepad1.a) g1APressed = true;
        else g1APressed = false;
        if (gamepad1.b) g1BPressed = true;
        else g1BPressed = false;

    }
}
