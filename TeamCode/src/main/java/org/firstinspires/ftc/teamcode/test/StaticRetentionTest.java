package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "StaticRetentionTest", group = "test")

public class StaticRetentionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Stored Time", TimeStorage.time);
        telemetry.addData("Current Time", System.currentTimeMillis());
        telemetry.update();

        waitForStart();

        TimeStorage.time = System.currentTimeMillis();
        telemetry.addData("New Time", TimeStorage.time);
        telemetry.update();

        while (!isStopRequested()) {}

    }

}
