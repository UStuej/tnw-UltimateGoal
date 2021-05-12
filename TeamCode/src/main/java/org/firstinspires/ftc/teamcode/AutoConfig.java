package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public static class Settings implements Serializable {

        // Unique long which specifies which version this class is
        public static final long serialVersionUID = 9145306921234518589L;

        // Settings to be stored as fields, and their default values

    }

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

        // Current config preset being modified
        Settings cfg;

        while (!isStopRequested()) {

            // Select or create new preset to modify
            if (cfgs != null) {

                telemetry.addLine("Modify existing config preset <A>, or create new config <B>?");
                telemetry.update();

                // Wait for A or B to be pressed
                do {
                    handleInput();
                } while (!gamepad1APressed && !gamepad1BPressed);

                if (gamepad1APressed) {

                    // Convert the unordered key Set of cfgs Map to an ordered array
                    String[] presets = (String[]) cfgs.keySet().toArray();

                    telemetry.clearAll();
                    telemetry.setAutoClear(false);

                    int presetIdx = 0;

                    {
                        telemetry.addLine("Select <B> a preset to modify...");
                        Telemetry.Item selector = telemetry.addData("<LB | RB>", presets[presetIdx]);
                        telemetry.update();

                        while (true) {

                            // Wait until a button is pressed
                            do {
                                handleInput();
                            } while (!gamepad1BHeld && !gamepad1LBPressed && !gamepad1RBPressed);

                            if (gamepad1BHeld) {  // Finish selection when B is pressed
                                break;
                            } else if (gamepad1LBPressed) {
                                presetIdx = presetIdx > 0
                                        ? presetIdx - 1
                                        : presets.length - 1;
                            } else if (gamepad1RBPressed) {
                                presetIdx = presetIdx < presets.length - 1
                                        ? presetIdx + 1
                                        : 0;
                            }

                            selector.setValue(presets[presetIdx]);
                            telemetry.update();

                        }
                    }

                    telemetry.setAutoClear(true);

                    cfg = cfgs.get(presets[presetIdx]);

                } else if (gamepad1BPressed) {
                    cfg = new Settings();
                }

            } else {
                cfg = new Settings();
            }

        }

        {
            int selectIdx = 0;

            // Select option and modify it
            while (true) {

                do {
                    handleInput();
                } while (!gamepad1LBPressed && !gamepad1RBPressed && !gamepad1UPPressed && !gamepad1DOWNPressed);

            }
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
