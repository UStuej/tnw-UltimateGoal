package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tnwutil.Settings;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "AutoConfig", group = "config")

public class AutoConfig extends LinearOpMode {

    private static final short CFG_NAME_LENGTH = 5;

    private boolean gamepad1AHeld = false;
    private boolean gamepad1APressed = false;
    private boolean gamepad1LBHeld = false;
    private boolean gamepad1LBPressed = false;
    private boolean gamepad1RBHeld = false;
    private boolean gamepad1RBPressed = false;
    private boolean gamepad1UPHeld = false;
    private boolean gamepad1UPPressed = false;
    private boolean gamepad1DOWNHeld = false;
    private boolean gamepad1DOWNPressed = false;


    @Override
    public void runOpMode() {

        // INIT

        Map<String, Settings> cfgs = null;

        // Read the already-existing configurations map if it exists
        {
            try (ObjectInputStream inputStream = new ObjectInputStream(new FileInputStream(new File(Environment.getDataDirectory(), "tnwAuto.cfg")))) {
                cfgs = (Map<String, Settings>) inputStream.readObject();
                telemetry.addLine("Configurations file found!");
            } catch (IOException | ClassNotFoundException | ClassCastException e) {
                telemetry.addLine("Configurations file not found; a new configuration preset will be created.");
            }
        }

        telemetry.addLine("Awaiting start...");
        telemetry.update();

        waitForStart();

        // START

        telemetry.setAutoClear(false);

        while (!isStopRequested()) {

            // Current config preset being modified
            Settings cfg = null;
            String cfgName = null;

            // Select or create new preset to modify
            if (cfgs != null) {

                telemetry.clearAll();
                telemetry.addLine("Modify existing config preset <LB>, create new config <RB>, or save changes <STOP>...");
                telemetry.update();

                // Wait for LB or RB to be pressed, or for the opmode to be stopped (resulting in saving the configurations)
                do {
                    handleInput();
                } while (!isStopRequested() && !gamepad1LBPressed && !gamepad1RBPressed);

                if (isStopRequested()) {
                    break;
                } else if (gamepad1LBPressed) {

                    // Convert the unordered key Set of cfgs Map to an ordered array
                    String[] presets = (String[]) cfgs.keySet().toArray();

                    short presetIdx = 0;

                    telemetry.clearAll();
                    telemetry.addLine("Select <A> a preset to modify <LB | RB>...");
                    Telemetry.Item selector = telemetry.addData("Preset", presets[presetIdx]);
                    telemetry.update();

                    while (true) {

                        // Wait until a button is pressed
                        do {
                            handleInput();
                        } while (!gamepad1APressed && !gamepad1LBPressed && !gamepad1RBPressed);

                        if (gamepad1APressed) {  // Finish selection when A is pressed
                            break;
                        } else if (gamepad1LBPressed) {
                            presetIdx = (short) (presetIdx > 0
                                    ? presetIdx - 1
                                    : presets.length - 1);
                        } else if (gamepad1RBPressed) {
                            presetIdx = (short) (presetIdx < presets.length - 1
                                    ? presetIdx + 1
                                    : 0);
                        }

                        selector.setValue(presets[presetIdx]);
                        telemetry.update();

                    }

                    cfgName = presets[presetIdx];
                    cfg = cfgs.get(cfgName);

                } else if (gamepad1RBPressed) {
                    cfg = new Settings();
                }

            } else {
                cfgs = new HashMap<>();
                // Create a new preset set to defaults if there is none
                cfg = new Settings();
            }

            {

                telemetry.clearAll();
                telemetry.addLine("Select a setting <LB | RB>, modify its value <UP | DOWN>, then save <A> when finished...");
                Telemetry.Item setting = telemetry.addData("Setting", "");
                Telemetry.Item description = telemetry.addData("Description", "");
                Telemetry.Item value = telemetry.addData("Value", "");

                short optionIdx = 0;

                // Controls to select an option and modify its value
                while (true) {

                    // Wait until a button is pressed
                    do {
                        handleInput();
                    } while (!gamepad1APressed && !gamepad1LBPressed && !gamepad1RBPressed && !gamepad1UPPressed && !gamepad1DOWNPressed);

                    if (gamepad1APressed) {  // Finish selection when A is pressed
                        break;
                    } else if (gamepad1LBPressed) {
                        optionIdx = (short) (optionIdx > 0
                                ? optionIdx - 1
                                : 2);  // Number of settings (option fields within Settings class)
                    } else if (gamepad1RBPressed) {
                        optionIdx = (short) (optionIdx < 2  // Number of settings (option fields within Settings class)
                                ? optionIdx + 1
                                : 0);
                    }

                    // Change to a new option to set, or modify the current one's value
                    switch (optionIdx) {

                        case 0:
                            // Flip the state of the boolean if either up or down is pressed
                            cfg.testBool = (gamepad1UPPressed || gamepad1DOWNPressed) != cfg.testBool;

                            setting.setValue("Test Bool");
                            description.setValue("booooooollelanena");
                            value.setValue(cfg.testBool);
                        break;

                        case 1:
                            // Increment or decrement the byte if either up or down is pressed, respectively
                            cfg.testByte = (byte) (
                                    gamepad1DOWNPressed
                                        ? cfg.testByte > 0
                                            ? cfg.testByte - 1
                                            : 3  // Max value allowed for byte
                                    : gamepad1UPPressed
                                        ? cfg.testByte < 3  // Max value allowed for byte
                                            ? cfg.testByte + 1
                                            : 0
                                    : cfg.testByte
                            );

                            setting.setValue("Test Byte");
                            description.setValue("btetrbyetytyb");
                            value.setValue(cfg.testByte);
                        break;

                    }

                    telemetry.update();

                }

            }

            // Ask for a name to save the preset as if a new one was created
            if (cfgName == null) {

                telemetry.clearAll();
                telemetry.addLine("To choose a name for the preset, move the cursor <LB | RB>, change the selected digit <UP | DOWN>, then save <A> when finished...");
                Telemetry.Item telName = telemetry.addData("Preset", "");

                // Array of digits in the name
                byte[] digits = new byte[CFG_NAME_LENGTH];
                short digitIdx = 0;

                while (true) {

                    // Name to be displayed on telemetry
                    StringBuilder displayName = new StringBuilder();

                    for (short i = 0; i < CFG_NAME_LENGTH; i++) {
                        if (i == digitIdx) {
                            displayName.append("|");
                            displayName.append(digits[digitIdx]);
                            displayName.append("|");
                        } else {
                            displayName.append(digits[digitIdx]);
                        }
                    }

                    telName.setValue(displayName);
                    telemetry.update();

                    // Wait until a button is pressed
                    do {
                        handleInput();
                    } while (!gamepad1APressed && !gamepad1LBPressed && !gamepad1RBPressed && !gamepad1UPPressed && !gamepad1DOWNPressed);

                    if (gamepad1APressed) {  // Finish selection when A is pressed
                        break;
                    } else if (gamepad1LBPressed) {
                        digitIdx = (short) (digitIdx > 0
                                ? digitIdx - 1
                                : CFG_NAME_LENGTH - 1);
                    } else if (gamepad1RBPressed) {
                        digitIdx = (short) (digitIdx < CFG_NAME_LENGTH - 1
                                ? digitIdx + 1
                                : 0);
                    } else if (gamepad1UPPressed) {
                        digits[digitIdx] = (byte) (digits[digitIdx] < 9
                                ? digits[digitIdx] + 1
                                : 0);
                    } else if (gamepad1DOWNPressed) {
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

            // Put the preset into the cfgs Map
            cfgs.put(cfgName, cfg);

        }

        // STOP

        // Save the new configurations map
        try (ObjectOutputStream outputStream = new ObjectOutputStream(new FileOutputStream(new File(Environment.getDataDirectory(), "tnwAuto.cfg")))) {
            outputStream.writeObject(cfgs);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    private void handleInput() {

        // Cache previous gamepad 1 inputs
        boolean gamepad1AWasHeld = gamepad1AHeld;
        boolean gamepad1LBWasHeld = gamepad1LBHeld;
        boolean gamepad1RBWasHeld = gamepad1RBHeld;
        boolean gamepad1UPWasHeld = gamepad1UPHeld;
        boolean gamepad1DOWNWasHeld = gamepad1DOWNHeld;

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1LBHeld = gamepad1.left_bumper;
        gamepad1RBHeld = gamepad1.right_bumper;
        gamepad1UPHeld = gamepad1.dpad_up;
        gamepad1DOWNHeld = gamepad1.dpad_down;

        // Determine if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;
        gamepad1LBPressed = !gamepad1LBWasHeld && gamepad1LBHeld;
        gamepad1RBPressed = !gamepad1RBWasHeld && gamepad1RBHeld;
        gamepad1UPPressed = !gamepad1UPWasHeld && gamepad1UPHeld;
        gamepad1DOWNPressed = !gamepad1DOWNWasHeld && gamepad1DOWNHeld;

    }

}
