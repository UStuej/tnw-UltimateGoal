package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MotorPositionFinder", group = "test")

public class MotorPositionFinder extends OpMode {

    //////////////////////////////////////////////////////////
    private final String[] SERVO_NAMES = {"servo0", "servo1", "servo2", "servo3", "servo4", "servo5", "servo6", "servo7", "servo8", "servo9", "servo10", "servo11"};
    private final String[] DCMOTOR_NAMES = {"motor0", "motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "motor7"};
    //////////////////////////////////////////////////////////

    private Servo[] servos = new Servo[SERVO_NAMES.length];
    private DcMotor[] dcMotors = new DcMotor[DCMOTOR_NAMES.length];

    private boolean rBumperReleased = false;
    private boolean lBumperReleased = false;
    private final double INCR = .01;
    private final double DCMOTOR_PWR_FACTOR = 0.25;
    private final double STARTING_POSITION = .5;

    private byte servoSelect = 0;
    private byte dcMotorSelect = 0;
    private double[] servoPositions = new double[SERVO_NAMES.length];
    //private int[] motorPositions = new int[DCMOTOR_NAMES.length];
    private double variablePosition = STARTING_POSITION;
    private double variablePower = 0.0;

    private boolean gamepad1AHeld = false;
    private boolean gamepad1BHeld = false;
    private boolean gamepad1XHeld = false;
    private boolean gamepad1YHeld = false;
    private boolean gamepad1LeftBumperHeld = false;
    private boolean gamepad1RightBumperHeld = false;
    private boolean gamepad1APressed = false;
    private boolean gamepad1BPressed = false;
    private boolean gamepad1XPressed = false;
    private boolean gamepad1YPressed = false;
    private boolean gamepad1LeftBumperPressed = false;
    private boolean gamepad1RightBumperPressed = false;

    {
        for (short i = 0; i < servoPositions.length; i++) {
            servoPositions[i] = STARTING_POSITION;
        }
    }

    @Override
    public void init() {

        for (short i = 0; i < SERVO_NAMES.length; i++)
            servos[i] = hardwareMap.get(Servo.class, SERVO_NAMES[i]);
        for (short i = 0; i < DCMOTOR_NAMES.length; i++)
            dcMotors[i] = hardwareMap.get(DcMotor.class, DCMOTOR_NAMES[i]);

        telemetry.addData("", "SERVOS WILL MOVE - BE READY...");
    }

    @Override
    public void start() {
        for (Servo servo : servos)
            servo.setPosition(STARTING_POSITION);
    }

    @Override
    public void loop() {

        handleInput();

        if (gamepad1LeftBumperPressed) {
            variablePosition += INCR;
            lBumperReleased = false;
        } else if (gamepad1RightBumperPressed) {
            variablePosition -= INCR;
            rBumperReleased = false;
        }

        variablePower = (gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_y) * DCMOTOR_PWR_FACTOR;
        //motorPositions[dcMotorSelect] = dcMotors[dcMotorSelect].getCurrentPosition();

        if (gamepad1APressed) {
            if (servoSelect == 0)
                servoSelect = (byte) (SERVO_NAMES.length - 1);
            else
                servoSelect--;
            variablePosition = servoPositions[servoSelect];
        } else if (gamepad1BPressed) {
            if (servoSelect == SERVO_NAMES.length - 1)
                servoSelect = 0;
            else
                servoSelect++;
            variablePosition = servoPositions[servoSelect];
        }

        if (gamepad1XPressed) {
            if (dcMotorSelect == 0)
                dcMotorSelect = (byte) (DCMOTOR_NAMES.length - 1);
            else
                dcMotorSelect--;
            variablePower = 0.0;
        } else if (gamepad1YPressed) {
            if (dcMotorSelect == DCMOTOR_NAMES.length - 1)
                dcMotorSelect = 0;
            else
                dcMotorSelect++;
            variablePower = 0.0;
        }

        servoPositions[servoSelect] = variablePosition;
        telemetry.addData("Selected Servo: ", SERVO_NAMES[servoSelect]);
        telemetry.addData( "Servo Position: ", variablePosition);

        telemetry.addData("Selected DC Motor: ", DCMOTOR_NAMES[dcMotorSelect]);
        telemetry.addData("DC Motor Power: ", variablePower);
        telemetry.addData("DC Motor Position: ", dcMotors[dcMotorSelect].getCurrentPosition());

        servos[servoSelect].setPosition(variablePosition);
        dcMotors[dcMotorSelect].setPower(variablePower);

        telemetry.update();
    }

    private void handleInput() {

        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;  // Whether or not the gamepad 1 a button was held
        boolean gamepad1BWasHeld = gamepad1BHeld;  // Whether or not the gamepad 1 b button was held
        boolean gamepad1XWasHeld = gamepad1XHeld;  // Whether or not the gamepad 1 x button was held
        boolean gamepad1YWasHeld = gamepad1YHeld;  // Whether or not the gamepad 1 y button was held
        boolean gamepad1LeftShoulderWasHeld = gamepad1LeftBumperHeld;  // Whether or not the gamepad 1 left shoulder button was held
        boolean gamepad1RightShoulderWasHeld = gamepad1RightBumperHeld;  // Whether or not the gamepad 1 left shoulder button was held

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1BHeld = gamepad1.b;
        gamepad1XHeld = gamepad1.x;
        gamepad1YHeld = gamepad1.y;
        gamepad1LeftBumperHeld = gamepad1.left_bumper;
        gamepad1RightBumperHeld = gamepad1.right_bumper;

        // Figure out if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;  // The button was just pressed if our previous value was false, and this one was true
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;
        gamepad1XPressed = !gamepad1XWasHeld && gamepad1XHeld;
        gamepad1YPressed = !gamepad1YWasHeld && gamepad1YHeld;
        gamepad1LeftBumperPressed = !gamepad1LeftShoulderWasHeld && gamepad1LeftBumperHeld;
        gamepad1RightBumperPressed = !gamepad1RightShoulderWasHeld && gamepad1RightBumperHeld;

    }
}
