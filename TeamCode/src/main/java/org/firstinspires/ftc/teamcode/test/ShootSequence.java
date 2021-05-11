package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shoot Sequence")

public class ShootSequence extends OpMode {

    private DcMotor ringElevator;
    private DcMotor intakeDrive;
    private DcMotorEx shoot;
    private Servo ringFinger;

    private boolean ringElevatorUp = false;

    private static int RING_ELEVATOR_UP_POSITION; // The position of the Ring Elevator when it is in the UP state
    private static int RING_ELEVATOR_DOWN_POSITION; // The position of the Ring Elevator when it is in the DOWN state
    private static final double RING_ELEVATOR_POWER = 1.0; // The power for the motor to use when running to its target position
    private final double RING_FINGER_IN_POSITION = 0.23;
    private final double RING_FINGER_OUT_POSITION = 0.75;
    private long targetTime;

    private int step = 0;


    @Override
    public void init() {
        ringElevator = hardwareMap.get(DcMotor.class, "ringElevator");
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        ringFinger = hardwareMap.get(Servo.class, "ringFinger");

        // Set Ring Elevator motor...
        RING_ELEVATOR_DOWN_POSITION = ringElevator.getCurrentPosition();
        RING_ELEVATOR_UP_POSITION = RING_ELEVATOR_DOWN_POSITION + 2075;
        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION);
        ringElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run mode
        telemetry.addData("WARNING: ", "Manually move 'ringElevator' to down position");
        shoot.setVelocityPIDFCoefficients(150, 7, 10, 0);
    }

    @Override
    public void start() {
        ringElevator.setPower(RING_ELEVATOR_POWER);
    }

    @Override
    public void loop() {
        intakeDrive.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if (gamepad1.a) {
            while (step != 17) {
                switch (step){
                    case 0:
                        targetTime = System.currentTimeMillis() + 1000;
                        ringFinger.setPosition(RING_FINGER_OUT_POSITION);
                        step = 1;
                        break;
                    case 1:
                        if (System.currentTimeMillis() >= targetTime) step = 2;
                        break;
                    case 2:
                        targetTime = System.currentTimeMillis() + 2000;
                        ringElevator.setTargetPosition(RING_ELEVATOR_UP_POSITION);
                        shoot.setVelocity(57 * 28);
                        step = 3;
                        break;
                    case 3:
                        if (System.currentTimeMillis() >= targetTime) step = 4;
                        break;
                    case 4:
                        targetTime = System.currentTimeMillis() + 500;
                        ringFinger.setPosition(RING_FINGER_IN_POSITION);
                        step = 5;
                        break;
                    case 5:
                        if (System.currentTimeMillis() >= targetTime) step = 6;
                        break;
                    case 6:
                        targetTime = System.currentTimeMillis() + 500;
                        ringFinger.setPosition(RING_FINGER_OUT_POSITION);
                        step = 7;
                        break;
                    case 7:
                        if (System.currentTimeMillis() >= targetTime) step = 8;
                        break;
                    case 8:
                        targetTime = System.currentTimeMillis() + 500;
                        ringFinger.setPosition(RING_FINGER_IN_POSITION);
                        step = 9;
                        break;
                    case 9:
                        if (System.currentTimeMillis() >= targetTime) step = 10;
                        break;
                    case 10:
                        targetTime = System.currentTimeMillis() + 500;
                        ringFinger.setPosition(RING_FINGER_OUT_POSITION);
                        step = 11;
                        break;
                    case 11:
                        if (System.currentTimeMillis() >= targetTime) step = 12;
                        break;
                    case 12:
                        targetTime = System.currentTimeMillis() + 500;
                        ringFinger.setPosition(RING_FINGER_IN_POSITION);
                        step = 13;
                        break;
                    case 13:
                        if (System.currentTimeMillis() >= targetTime) step = 14;
                        break;
                    case 14:
                        targetTime = System.currentTimeMillis() + 500;
                        ringFinger.setPosition(RING_FINGER_OUT_POSITION);
                        step = 15;
                        break;
                    case 15:
                        if (System.currentTimeMillis() >= targetTime) step = 16;
                        break;
                    case 16:
                        shoot.setPower(0);
                        ringElevator.setTargetPosition(RING_ELEVATOR_DOWN_POSITION);
                        step = 17;
                        break;
                }
            }
        }

        step = 0;

    }
}
