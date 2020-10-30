package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.DcMotorControl;

public class Main extends OpMode {

    // Declare drive motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Declare intake motors
    private DcMotor intakeDrive;

    // Declare Wobble Goal manipulation motors
    private DcMotor WGLift;
    private Servo WGPickup;
    private Servo WGShoulder;
    private Servo WGClaw;

    // Declare drive power
    double frontLeftDrivePower;
    double frontRightDrivePower;
    double backLeftDrivePower;
    double backRightDrivePower;

    // Declare chassis motion variables
    double currentPower;
    double vertical;
    double horizontal;
    double rotation;

    @Override
    public void init(){

        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Initialize intake motors
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

        // Initialize Wobble Goal manipulation motors
        WGLift = hardwareMap.get(DcMotor.class, "WGLift");
        WGPickup = hardwareMap.get(Servo.class, "WGPickup");
        WGShoulder = hardwareMap.get(Servo.class, "WGShoulder");
        WGClaw = hardwareMap.get(Servo.class, "WGClaw");

        // Set drive motor directions
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop(){

        // FIXME Switch to a non-polling system if possible
        currentPower = 0.0; // FIXME Replace 0.0 with current motor power
        vertical = DcMotorControl.motorIncrControl(-gamepad1.left_stick_y, currentPower);
        horizontal = DcMotorControl.motorIncrControl(gamepad1.left_stick_x, currentPower);
        rotation = DcMotorControl.motorIncrControl(gamepad1.right_stick_x, currentPower);

    }

}
