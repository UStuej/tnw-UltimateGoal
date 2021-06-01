package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.tnwutil.AutoSettings;
import org.firstinspires.ftc.teamcode.tnwutil.GamepadInputHandler;

import java.util.Map;

@Autonomous(preselectTeleOp = "TeleOp99Mark2")
public class TNWStateCupAuto extends LinearOpMode {

    SampleMecanumDrive drive;

    AutoSettings config;
    boolean blueAlliance;

    @Override
    public void runOpMode() throws InterruptedException {

        // INIT

        drive = new SampleMecanumDrive(hardwareMap);

        if (userGetSettings()) return;

        telemetry.addLine("Awaiting start...");
        telemetry.update();
        waitForStart();

        // START

    }

    private boolean userGetSettings() {

        telemetry.setAutoClear(false);

        Map<String, AutoSettings> configs = AutoConfig.readConfigsFile();
        AutoSettings config;
        GamepadInputHandler input = new GamepadInputHandler(gamepad1);

        if (configs != null) {
            String cfgName = AutoConfig.userSelectPreset(configs, input, telemetry, this::isStopRequested);
            if (cfgName == null) return true;
            config = configs.get(cfgName);
        } else {
            config = new AutoSettings();
        }
        if (AutoConfig.userModifyConfig(config, input, telemetry, this::isStopRequested)) return true;
        this.config = config;

        // Get alliance color from user
        telemetry.addData("<X>", "Blue Alliance");
        telemetry.addData("<B>", "Red Alliance");
        telemetry.update();
        do {
            input.update();
        } while (!isStopRequested() && !input.isXPressed() && !input.isBPressed());
        if (isStopRequested()) return true;
        blueAlliance = input.isXPressed();
        telemetry.clearAll();
        telemetry.update();
        telemetry.setAutoClear(true);

        return false;

    }

}
