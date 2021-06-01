package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.tnwutil.AutoSettings;
import org.firstinspires.ftc.teamcode.tnwutil.GamepadInputHandler;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

@TeleOp(group = "config")
public class AutoConfig extends LinearOpMode {

    // USER DEFINED CONSTANTS
    private static final short CFG_NAME_LENGTH = 5;  // Length of assignable names (array of digits) to configuration preset

    @Override
    public void runOpMode() {
        
        // INIT

        Map<String, AutoSettings> configs = readConfigsFile();
        GamepadInputHandler input = new GamepadInputHandler(gamepad1);

        if (configs != null) telemetry.addLine("Configurations file found!");
        else telemetry.addLine("Configurations file not found; a new configuration preset will be created.");

        telemetry.addLine("Awaiting start...");
        telemetry.update();

        waitForStart();

        // START

        telemetry.setAutoClear(false);

        while (true) {

            // Current config preset being modified
            AutoSettings config = null;
            String cfgName = null;

            // Select or create new preset to modify
            if (configs != null) {

                telemetry.clearAll();
                telemetry.addLine("Modify existing config preset <LB>, create new config preset <RB>, clear existing configurations <DOWN>, or save changes <A>...");
                telemetry.update();

                // Wait for LB or RB to be pressed, or for the OpMode to be stopped (resulting in saving the configurations)
                do {
                    input.update();
                } while (!isStopRequested() && !input.isAPressed() && !input.isLeftBumperPressed() && !input.isRightBumperPressed() && !input.isDpadDownPressed());

                if (isStopRequested()) {  // Exit the program if the stop button is pressed
                    return;
                } else if (input.isAPressed()) {  // Finish and save when A is pressed
                    break;
                } else if (input.isLeftBumperPressed()) {
                    cfgName = userSelectPreset(configs, input, telemetry, this::isStopRequested);
                    if (cfgName == null) return;
                    config = configs.get(cfgName);
                } else if (input.isRightBumperPressed()) {
                    config = new AutoSettings();
                } else if (input.isDpadDownPressed()) {

                    telemetry.clearAll();
                    telemetry.addLine("WARNING:  This will reset all existing configuration presets on the next save!");
                    telemetry.addLine("Proceed <UP>, or go back to menu <A>...");
                    telemetry.update();

                    // Wait for a button to be pressed
                    do {
                        input.update();
                    } while (!isStopRequested() && !input.isDpadUpPressed() && !input.isAPressed());

                    if (isStopRequested()) {  // Exit the program if the stop button is pressed
                        return;
                    } else if (input.isDpadUpPressed()) {
                        configs = null;
                    }

                    continue;

                }

            } else {
                configs = new HashMap<>();
                // Create a new preset set to defaults if there is none
                config = new AutoSettings();
            }

            if (userModifyConfig(config, input, telemetry, this::isStopRequested)) return;

            // Allow the user to enter a name to save the preset as if a new one was created
            if (cfgName == null) {

                telemetry.clearAll();
                telemetry.addLine("To choose a name for the preset, move the cursor <LB | RB>, change the selected digit <UP | DOWN>, then save <A> when finished...");
                Telemetry.Item telName = telemetry.addData("Preset", "");

                // Array of digits in the name
                byte[] digits = new byte[CFG_NAME_LENGTH];
                short digitIdx = 0;

                while (true) {

                    // Name to be displayed on telemetry
                    StringBuilder displayName = new StringBuilder(CFG_NAME_LENGTH + 2);

                    for (short i = 0; i < CFG_NAME_LENGTH; i++) {
                        if (i == digitIdx) {
                            displayName.append('[');
                            displayName.append(digits[i]);
                            displayName.append(']');
                        } else {
                            displayName.append(digits[i]);
                        }
                    }

                    telName.setValue(displayName);
                    telemetry.update();

                    // Wait for a button to be pressed
                    do {
                        input.update();
                    } while (!isStopRequested() && !input.isAPressed() && !input.isLeftBumperPressed() && !input.isRightBumperPressed() && !input.isDpadUpPressed() && !input.isDpadDownPressed());

                    if (isStopRequested()) {  // Exit the program if the stop button is pressed
                      return;
                    } else if (input.isAPressed()) {  // Finish selection when A is pressed
                        break;
                    } else if (input.isLeftBumperPressed()) {
                        digitIdx = (short) (digitIdx > 0
                                ? digitIdx - 1
                                : CFG_NAME_LENGTH - 1);
                    } else if (input.isRightBumperPressed()) {
                        digitIdx = (short) (digitIdx < CFG_NAME_LENGTH - 1
                                ? digitIdx + 1
                                : 0);
                    } else if (input.isDpadUpPressed()) {
                        digits[digitIdx] = (byte) (digits[digitIdx] < 9
                                ? digits[digitIdx] + 1
                                : 0);
                    } else if (input.isDpadDownPressed()) {
                        digits[digitIdx] = (byte) (digits[digitIdx] > 0
                                ? digits[digitIdx] - 1
                                : 9);
                    }

                }

                // Write digit sequence to cfgName
                StringBuilder name = new StringBuilder();
                for (short i = 0; i < CFG_NAME_LENGTH; i++) {
                    name.append(digits[i]);
                }
                cfgName = name.toString();

            }

            // Put the preset into the configs Map
            configs.put(cfgName, config);

        }

        // Save the new configurations map
        try (ObjectOutputStream outputStream = new ObjectOutputStream(new FileOutputStream(new File(AppUtil.getDefContext().getFilesDir(), "tnwAuto.cfg")))) {
            outputStream.writeObject(configs);
        } catch (IOException e) {
            telemetry.addLine("Could not write to configurations file!  No changes made since the previous save will be saved.");
            telemetry.addLine("[ ; _ ;]");
            sleep(10000);
            return;
        }

        telemetry.clearAll();
        telemetry.addLine("Configurations successfully saved.  Goodbye...");
        telemetry.update();

        sleep(1000);

    }

    public static Map<String, AutoSettings> readConfigsFile() {

        // Read the already-existing configurations map if it exists
        try (ObjectInputStream inputStream = new ObjectInputStream(new FileInputStream(new File(AppUtil.getDefContext().getFilesDir(), "tnwAuto.cfg")))) {
            return (Map<String, AutoSettings>) inputStream.readObject();
        } catch (IOException | ClassNotFoundException | ClassCastException e) {
            return null;
        }

    }

    public static String userSelectPreset(Map<String, AutoSettings> configs, GamepadInputHandler input, Telemetry telemetry, BooleanSupplier forceStop){

        // Convert the unordered key Set of configs Map to an ordered array
        String[] presets = Arrays.copyOf(configs.keySet().toArray(), configs.size(), String[].class);

        short presetIdx = 0;

        telemetry.clearAll();
        telemetry.addLine("Select <A> a preset to view/modify <LB | RB>...");
        Telemetry.Item selector = telemetry.addData("Preset", presets[presetIdx]);
        telemetry.update();

        while (true) {

            // Wait for a button to be pressed
            do {
                input.update();
            } while (!forceStop.getAsBoolean() && !input.isAPressed() && !input.isLeftBumperPressed() && !input.isRightBumperPressed());

            if (forceStop.getAsBoolean()) {
                return null;
            } else if (input.isAPressed()) {  // Finish selection when A is pressed
                break;
            } else if (input.isLeftBumperPressed()) {
                presetIdx = (short) (presetIdx > 0
                        ? presetIdx - 1
                        : presets.length - 1);
            } else if (input.isRightBumperPressed()) {
                presetIdx = (short) (presetIdx < presets.length - 1
                        ? presetIdx + 1
                        : 0);
            }

            selector.setValue(presets[presetIdx]);
            telemetry.update();

        }

        telemetry.clearAll();
        telemetry.update();

        return presets[presetIdx];

    }

    public static boolean userModifyConfig(AutoSettings config, GamepadInputHandler input, Telemetry telemetry, BooleanSupplier forceStop) {

        telemetry.clearAll();
        telemetry.addData("---", "Select a setting <LB | RB>, modify its value <LEFT | RIGHT>, then save <A> when finished...");
        Telemetry.Item[] settings = {
                telemetry.addData("Starting Line", () -> config.innerStartingLine ? "Inner" : "Outer"),
                telemetry.addData("Starter Stack", () -> config.starterStack ? "YES" : "no"),
                telemetry.addData("Power Shots", () -> config.powerShots ? "YES" : "no"),
                telemetry.addData("Deliver Wobble", () -> config.deliverWobble ? "YES" : "no"),
                telemetry.addData("Park", () -> config.park ? "YES" : "no"),
                telemetry.addData("Parking Location", () -> config.parkingLocation + " tiles")
        };

        short optionIdx = 0;
        String storedCaption = settings[optionIdx].getCaption();

        settings[optionIdx].setCaption("> " + storedCaption.toUpperCase());
        input.update();

        // Controls to select an option and modify its value
        while (true) {

            if (forceStop.getAsBoolean()) {  // Exit the program if the stop button is pressed
                return true;
            } else if (input.isAPressed()) {  // Finish selection when A is pressed
                break;
            } else if (input.isLeftBumperPressed()) {
                settings[optionIdx].setCaption(storedCaption);
                optionIdx = (short) (optionIdx > 0
                        ? optionIdx - 1
                        : AutoSettings.LENGTH - 1);
                storedCaption = settings[optionIdx].getCaption();
                settings[optionIdx].setCaption("> " + storedCaption.toUpperCase());
            } else if (input.isRightBumperPressed()) {
                settings[optionIdx].setCaption(storedCaption);
                optionIdx = (short) (optionIdx < AutoSettings.LENGTH - 1
                        ? optionIdx + 1
                        : 0);
                storedCaption = settings[optionIdx].getCaption();
                settings[optionIdx].setCaption("> " + storedCaption.toUpperCase());
            }

            // Change to a new option to set, or modify the current one's value
            switch (optionIdx) {

                case 0:
                    config.innerStartingLine = (input.isDpadLeftPressed() || input.isDpadRightPressed()) != config.innerStartingLine;
                break;

                case 1:
                    config.starterStack = (input.isDpadLeftPressed() || input.isDpadRightPressed()) != config.starterStack;
                break;

                case 2:
                    config.powerShots = (input.isDpadLeftPressed() || input.isDpadRightPressed()) != config.powerShots;
                break;

                case 3:
                    config.deliverWobble = (input.isDpadLeftPressed() || input.isDpadRightPressed()) != config.deliverWobble;
                break;

                case 4:
                    config.park = (input.isDpadLeftPressed() || input.isDpadRightPressed()) != config.park;
                break;

                case 5:
                    if (input.isDpadLeftPressed()) {
                        config.parkingLocation = (byte) (optionIdx > 0
                                ? config.parkingLocation - 1
                                : 4);
                    } else if (input.isDpadRightPressed()) {
                        config.parkingLocation = (byte) (optionIdx < 4
                                ? config.parkingLocation + 1
                                : 0);
                    }
                break;

            }

            telemetry.update();

            // Wait for a button to be pressed
            do {
                input.update();
            } while (!forceStop.getAsBoolean() && !input.isAPressed() && !input.isLeftBumperPressed() && !input.isRightBumperPressed() && !input.isDpadLeftPressed() && !input.isDpadRightPressed());

        }

        telemetry.clearAll();
        telemetry.update();

        return false;

    }

}
