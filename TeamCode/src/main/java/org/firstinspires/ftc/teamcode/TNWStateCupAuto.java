package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.tnwutil.AutoSettings;
import org.firstinspires.ftc.teamcode.tnwutil.GamepadInputHandler;

import java.util.Map;

@Autonomous(preselectTeleOp = "TeleOp99Mark2")
public class TNWStateCupAuto extends LinearOpMode {

    SampleMecanumDrive drive;

    AutoSettings config;
    boolean cfgIsBlueAlliance;
    long cfgStartDelay;

    @Override
    public void runOpMode() throws InterruptedException {

        // INIT

        drive = new SampleMecanumDrive(hardwareMap);

        if (userGetSettings()) return;

        telemetry.addLine("Awaiting start...");
        telemetry.update();
        waitForStart();

        // START

        // Debug to show values which are being used
        telemetry.addData("innerStartingLine", config.innerStartingLine);
        telemetry.addData("starterStack", config.starterStack);
        telemetry.addData("deliverWobble", config.deliverWobble);
        telemetry.addData("park", config.park);
        telemetry.addData("parkingLocation", config.parkingLocation);
        telemetry.addData("isBlueAlliance", cfgIsBlueAlliance);
        telemetry.addData("startDelay", cfgStartDelay);
        telemetry.update();

        sleep(30000);

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
        cfgIsBlueAlliance = input.isXPressed();
        telemetry.clearAll();

        // Get start delay from user
        // DEV: Could be placed within a scope (temporary variable declared), but is not since this is the last "block" of code in this method, and can be grouped into its exit gc.
        telemetry.addLine("<LEFT | RIGHT> to modify, <A> to continue...");
        short dispStartDelay = 0;
        Telemetry.Item itemStartDelay = telemetry.addData("Start Delay", "");
        while (true) {
            itemStartDelay.setValue(dispStartDelay + " seconds");
            telemetry.update();
            do {
                input.update();
            } while (!isStopRequested() && !input.isAPressed() && !input.isDpadLeftPressed() && !input.isDpadRightPressed());
            if (isStopRequested()) {
                return true;
            } else if (input.isAPressed()) {
                break;
            } else if (input.isDpadLeftPressed()) {
                dispStartDelay = (short) (dispStartDelay > 0
                        ? dispStartDelay - 1
                        : 0);
            } else if (input.isDpadRightPressed()) {
                dispStartDelay++;
            }
        }
        cfgStartDelay = dispStartDelay * 1000L;
        telemetry.clearAll();
        // DEV: telemetry.update() is not called, since it will be directly after this method is.

        telemetry.setAutoClear(true);

        return false;

    }

}
