package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "SlingshotControl")

// A fun, experimental OpMode. Think Angry Birds, but without gravity and you bounce off walls.
// It's actually closer to "What The Golf" if you've played that

public class SlingshotControl extends OpMode {
    double vertical = 0.0;  // Offset axes for the robot control
    double horizontal = 0.0;
    double rotation = 0.0;

    private Pose2d currentPose = PoseStorage.currentPose;
    private Pose2d wobbleGoal1Position = PoseStorage.wobbleGoal1RedPosition;
    private Pose2d wobbleGoal2Position = PoseStorage.wobbleGoal2RedPosition;
    private SampleMecanumDrive drive;

    private Vector2d inputVector;
    private Vector2d lastInputVector;

    double inputAngle;

    private Pose2d currentVelocity;

    private long currentTime;
    private long deltaTime;
    private long lastTime;

    private double velocityDampeningFactor = 1.25;  // 1 is no dampening, more than one is friction, and less than 1 will make it impossible to stop

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);  // The roadrunner representation of the robot, used for automatic rotation correction
        drive.setPoseEstimate(currentPose);

        gamepad1.setJoystickDeadzone(0.2f);

        currentTime = System.currentTimeMillis();
        deltaTime = (long) (1000 * (1.0/60.0));
    }

    public void handleInput() {
        lastTime = currentTime;
        currentTime = System.currentTimeMillis();
        deltaTime = lastTime - currentTime;

        inputVector = new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y);

        if (Math.abs(Math.sqrt(Math.pow(lastInputVector.getX(), 2) + Math.pow(lastInputVector.getY(), 2)) - Math.sqrt(Math.pow(inputVector.getX(), 2) + Math.pow(inputVector.getY(), 2))) / (double) deltaTime >= 0.0014) {  // Measure joystick acceleration and test for "jerks" which will release the slingshot opposite the angle of the joystick
            inputAngle = lastInputVector.unaryMinus().angle();
            currentVelocity = new Pose2d(lastInputVector.unaryMinus(), currentVelocity.getHeading());  // Actually slingshot the bot
        }
        else {
            currentVelocity = currentVelocity.div((velocityDampeningFactor * (1 + Math.sqrt(Math.pow(inputVector.getX(), 2) + Math.pow(inputVector.getY(), 2)))) * deltaTime);  // Dampen movement based on joystick magnitude and elapsed time
        }

        if (Math.sqrt(Math.pow(inputVector.getX(), 2) + Math.pow(inputVector.getY(), 2)) > 0.0) {
            rotation = inputAngle - currentPose.getHeading();  // Rotate based on current joystick input if the magnitude is above 0
        }

        //Vector2d directionalVector = new Vector2d(horizontal, vertical);
        //directionalVector = directionalVector.rotated(-drive.getPoseEstimate().getHeading());

        drive.setDrivePower(
                new Pose2d(
                        currentVelocity.getX(),
                        currentVelocity.getY(),
                        rotation
                )
        );

        // Update all roadrunner stuff (odometry, etc.)
        // Do not that this method completely evades acceleration checks and the like, leaving all calculations to roadrunner
        drive.update();

        // Read pose
        currentPose = drive.getPoseEstimate();

        currentVelocity = drive.getPoseVelocity();
    }

    @Override
    public void loop() {

    }
}