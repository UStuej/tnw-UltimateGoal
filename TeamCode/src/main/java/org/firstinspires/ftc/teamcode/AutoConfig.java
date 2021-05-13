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
import java.io.Serializable;
import java.util.Map;

@TeleOp(name = "AutoConfig", group = "config")

public class AutoConfig extends LinearOpMode {

    private boolean gamepad1AHeld = false;
    private boolean gamepad1APressed = false;
    private boolean gamepad1BHeld = false;
    private boolean gamepad1BPressed = false;
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

        // Read the already-existing configurations map{, or create a new one set to defaults if there is none}
        Map<String, Settings> cfgs = null;
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

        while (!isStopRequested()) {

            // Current config preset being modified
            Settings cfg = null;

            // Select or create new preset to modify
            if (cfgs != null) {

                telemetry.addLine("Modify existing config preset <A>, or create new config <B>?");
                telemetry.update();

                // Wait for A or B to be pressed, or for the opmode to be stopped (resulting in saving the configurations)
                do {
                    handleInput();
                } while (!isStopRequested() && !gamepad1APressed && !gamepad1BPressed);

                if (isStopRequested()) {
                    break;
                } else if (gamepad1APressed) {

                    // Convert the unordered key Set of cfgs Map to an ordered array
                    String[] presets = (String[]) cfgs.keySet().toArray();

                    short presetIdx = 0;

                    {
                        telemetry.addLine("Select <B> a preset to modify...");
                        Telemetry.Item selector = telemetry.addData("<LB | RB>", presets[presetIdx]);
                        telemetry.update();
                        telemetry.setAutoClear(false);

                        while (true) {

                            // Wait until a button is pressed
                            do {
                                handleInput();
                            } while (!gamepad1BHeld && !gamepad1LBPressed && !gamepad1RBPressed);

                            if (gamepad1BHeld) {  // Finish selection when B is pressed
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

                        telemetry.setAutoClear(true);
                    }

                    cfg = cfgs.get(presets[presetIdx]);

                } else if (gamepad1BPressed) {
                    cfg = new Settings();
                }

            } else {
                cfg = new Settings();
            }

            {

                telemetry.addLine("Select a setting <LB | RB> and modify its value <UP | DOWN>, then exit <A> when finished...");
                Telemetry.Item setting = telemetry.addData("Setting", "");
                Telemetry.Item description = telemetry.addData("Description", "");
                Telemetry.Item value = telemetry.addData("Value", "");
                telemetry.setAutoClear(false);

                short optionIdx = 0;

                // Controls to select an option and modify its value
                while (true) {

                    // Wait until a button is pressed
                    do {
                        handleInput();
                    } while (!gamepad1AHeld && !gamepad1LBPressed && !gamepad1RBPressed && !gamepad1UPPressed && !gamepad1DOWNPressed);

                    if (gamepad1AHeld) {  // Finish selection when A is pressed
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
                            cfg.testBool = (gamepad1UPPressed || gamepad1DOWNPressed) != cfg.testBool;  // Flip the state of the boolean if either up or down is pressed

                            setting.setValue("Test Bool");
                            description.setValue("booooooollelanena");
                            value.setValue(cfg.testBool);
                        break;

                        case 1:
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

            // TODO Ask for a name to save the preset as if new, and put the preset into the cfgs Map

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
        boolean gamepad1BWasHeld = gamepad1BHeld;
        boolean gamepad1LBWasHeld = gamepad1LBHeld;
        boolean gamepad1RBWasHeld = gamepad1RBHeld;
        boolean gamepad1UPWasHeld = gamepad1UPHeld;
        boolean gamepad1DOWNWasHeld = gamepad1DOWNHeld;

        // Get new values from the actual gamepad 1
        gamepad1AHeld = gamepad1.a;
        gamepad1BHeld = gamepad1.b;
        gamepad1LBHeld = gamepad1.left_bumper;
        gamepad1RBHeld = gamepad1.right_bumper;
        gamepad1UPHeld = gamepad1.dpad_up;
        gamepad1DOWNHeld = gamepad1.dpad_down;

        // Determine if they were just pressed using our cached inputs and the new ones
        gamepad1APressed = !gamepad1AWasHeld && gamepad1AHeld;
        gamepad1BPressed = !gamepad1BWasHeld && gamepad1BHeld;
        gamepad1LBPressed = !gamepad1LBWasHeld && gamepad1LBHeld;
        gamepad1RBPressed = !gamepad1RBWasHeld && gamepad1RBHeld;
        gamepad1UPPressed = !gamepad1UPWasHeld && gamepad1UPHeld;
        gamepad1DOWNPressed = !gamepad1DOWNWasHeld && gamepad1DOWNHeld;

    }

}
