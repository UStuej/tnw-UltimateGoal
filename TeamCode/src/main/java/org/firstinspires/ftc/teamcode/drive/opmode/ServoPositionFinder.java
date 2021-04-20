package org.firstinspires.ftc.teamcode.drive.opmode;

import java.lang.String;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPositionFinder", group = "calibration")

public class ServoPositionFinder extends OpMode {

    private Servo servo0;
    private Servo servo1;

    //////////////////////////////////////////////////////////
    final String SERVO_0_NAME = "servo12";
    final String SERVO_1_NAME = "servo7";
    //////////////////////////////////////////////////////////

    boolean initialization = true;
    boolean rBumperReleased = false;
    boolean lBumperReleased = false;
    final double INCR = .01;
    final double STARTING_POSITION = .5;

    byte servoSelect = -1;
    double servo0Position;
    double servo1Position;
    double variablePosition = STARTING_POSITION;

    @Override
    public void init() {

        servo0 = hardwareMap.get(Servo.class, SERVO_0_NAME);
        servo1 = hardwareMap.get(Servo.class, SERVO_1_NAME);

        telemetry.addData("", "SERVOS WILL MOVE - BE READY...");
    }

    @Override
    public void loop() {
        if (initialization) {  // initialization to be run only once at the start of the program to remove surprise servo movements when changing controlled servos
            servo0Position = STARTING_POSITION;
            servo1Position = STARTING_POSITION;
            initialization = false;
        }

        if (!gamepad1.left_bumper) lBumperReleased = true;
        if (!gamepad1.right_bumper) rBumperReleased = true;

        if (servoSelect != -1) {  // only runs once a servo has been selected
            if (gamepad1.left_bumper && lBumperReleased) { variablePosition += INCR; lBumperReleased = false; }
            else if (gamepad1.right_bumper && rBumperReleased) { variablePosition -= INCR; rBumperReleased = false; }
        }

        if (gamepad1.a) { servoSelect = 0; variablePosition = servo0Position; }
        else if (gamepad1.b) { servoSelect = 1; variablePosition = servo1Position; }

        switch (servoSelect) {
            case 0: servo0Position = variablePosition; telemetry.addData("Selected Servo: ", SERVO_0_NAME); break;
            case 1: servo1Position = variablePosition; telemetry.addData("Selected Servo: ", SERVO_1_NAME); break;
            default: telemetry.addData("Selected Servo: ", "No servo selected..."); break;
        }

        telemetry.addData("Press 'A' for ", SERVO_0_NAME);
        telemetry.addData("Press 'B' for ", SERVO_1_NAME);
        telemetry.addData(SERVO_0_NAME + "Position: ", servo0Position);
        telemetry.addData(SERVO_1_NAME + "Position: ", servo1Position);

        servo0.setPosition(servo0Position);
        servo1.setPosition(servo1Position);
        telemetry.update();
    }
}
