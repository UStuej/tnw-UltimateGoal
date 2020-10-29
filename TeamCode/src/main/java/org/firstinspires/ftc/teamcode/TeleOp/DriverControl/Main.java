package org.firstinspires.ftc.teamcode.TeleOp.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.TeleOp.DcMotorControl;

public class Main extends OpMode{

    //Declaring Drive Motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    //Declaring Intake Motors
    private DcMotor intakeDrive;

    //Declaring Wobble Goal Manipulation Motors
    private DcMotor WGLift;
    private Servo WGPickup;
    private Servo WGShoulder;
    private Servo WGClaw;

    //Declaring Drive Power Variables
    float frontLeftDrivePower;
    float frontRightDrivePower;
    float backLeftDrivePower;
    float backRightDrivePower;

    //Declaring Chassis Motion Variables
    float vertical;
    float horizontal;
    float rotation;

    @Override
    public void init(){

        //Initializing Drive Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        //Initializing Intake Motors
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

        //Initializing Wobble Goal Manipulation Motors
        WGLift = hardwareMap.get(DcMotor.class, "WGLift");
        WGPickup = hardwareMap.get(Servo.class, "WGPickup");
        WGShoulder = hardwareMap.get(Servo.class, "WGShoulder");
        WGClaw = hardwareMap.get(Servo.class, "WGClaw");

        //Setting Drive Motor Directions
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop(){

        vertical = DcMotorControl.motorControl(-1*gamepad1.left_stick_y);
        horizontal = DcMotorControl.motorControl(gamepad1.left_stick_x);
        rotation = DcMotorControl.motorControl(gamepad1.right_stick_x);

        
    }

    @Override
    public void stop(){

    }
}
