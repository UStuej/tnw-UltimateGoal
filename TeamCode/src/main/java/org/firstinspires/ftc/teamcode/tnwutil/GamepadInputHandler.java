package org.firstinspires.ftc.teamcode.tnwutil;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A class which stores different properties of button states of a {@link Gamepad} over time (calls to {@link #update()}).
 */
public class GamepadInputHandler {

    private Gamepad gamepad;

    // States of gamepad buttons
    private boolean isAHeld;
    private boolean isAPressed;
    private boolean isAReleased;
    private boolean isBHeld;
    private boolean isBPressed;
    private boolean isBReleased;
    private boolean isXHeld;
    private boolean isXPressed;
    private boolean isXReleased;
    private boolean isYHeld;
    private boolean isYPressed;
    private boolean isYReleased;
    private boolean isDpadUpHeld;
    private boolean isDpadUpPressed;
    private boolean isDpadUpReleased;
    private boolean isDpadDownHeld;
    private boolean isDpadDownPressed;
    private boolean isDpadDownReleased;
    private boolean isDpadLeftHeld;
    private boolean isDpadLeftPressed;
    private boolean isDpadLeftReleased;
    private boolean isDpadRightHeld;
    private boolean isDpadRightPressed;
    private boolean isDpadRightReleased;
    private boolean isLeftBumperHeld;
    private boolean isLeftBumperPressed;
    private boolean isLeftBumperReleased;
    private boolean isRightBumperHeld;
    private boolean isRightBumperPressed;
    private boolean isRightBumperReleased;
    private boolean isLeftStickHeld;
    private boolean isLeftStickPressed;
    private boolean isLeftStickReleased;
    private boolean isRightStickHeld;
    private boolean isRightStickPressed;
    private boolean isRightStickReleased;
    private boolean isBackHeld;
    private boolean isBackPressed;
    private boolean isBackReleased;
    private boolean isStartHeld;
    private boolean isStartPressed;
    private boolean isStartReleased;

    public GamepadInputHandler(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {

        // Cache previous gamepad button inputs
        boolean wasAHeld = isAHeld;
        boolean wasBHeld = isBHeld;
        boolean wasXHeld = isXHeld;
        boolean wasYHeld = isYHeld;
        boolean wasDpadUpHeld = isDpadUpHeld;
        boolean wasDpadDownHeld = isDpadDownHeld;
        boolean wasDpadLeftHeld = isDpadLeftHeld;
        boolean wasDpadRightHeld = isDpadRightHeld;
        boolean wasLeftBumperHeld = isLeftBumperHeld;
        boolean wasRightBumperHeld = isRightBumperHeld;
        boolean wasLeftStickHeld = isLeftStickHeld;
        boolean wasRightStickHeld = isRightStickHeld;
        boolean wasBackHeld = isBackHeld;
        boolean wasStartHeld = isStartHeld;

        // Get new button values from the actual gamepad
        isAHeld = gamepad.a;
        isBHeld = gamepad.b;
        isXHeld = gamepad.x;
        isYHeld = gamepad.y;
        isDpadUpHeld = gamepad.dpad_up;
        isDpadDownHeld = gamepad.dpad_down;
        isDpadLeftHeld = gamepad.dpad_left;
        isDpadRightHeld = gamepad.dpad_right;
        isLeftBumperHeld = gamepad.left_bumper;
        isRightBumperHeld = gamepad.right_bumper;
        isLeftStickHeld = gamepad.left_stick_button;
        isRightStickHeld = gamepad.right_stick_button;
        isBackHeld = gamepad.back;
        isStartHeld = gamepad.start;

        // Determine if the buttons were just pressed using our cached inputs and the new ones
        isAPressed = !wasAHeld && isAHeld;
        isBPressed = !wasBHeld && isBHeld;
        isXPressed = !wasXHeld && isXHeld;
        isYPressed = !wasYHeld && isYHeld;
        isDpadUpPressed = !wasDpadUpHeld && isDpadUpHeld;
        isDpadDownPressed = !wasDpadDownHeld && isDpadDownHeld;
        isDpadLeftPressed = !wasDpadLeftHeld && isDpadLeftHeld;
        isDpadRightPressed = !wasDpadRightHeld && isDpadRightHeld;
        isLeftBumperPressed = !wasLeftBumperHeld && isLeftBumperHeld;
        isRightBumperPressed = !wasRightBumperHeld && isRightBumperHeld;
        isLeftStickPressed = !wasLeftStickHeld && isLeftStickHeld;
        isRightStickPressed = !wasRightStickHeld && isRightStickHeld;
        isBackPressed = !wasBackHeld && isBackHeld;
        isStartPressed = !wasStartHeld && isStartHeld;

        // Determine if the buttons were just released using our cached inputs and the new ones
        isAReleased = wasAHeld && !isAHeld;
        isBReleased = wasBHeld && !isBHeld;
        isXReleased = wasXHeld && !isXHeld;
        isYReleased = wasYHeld && !isYHeld;
        isDpadUpReleased = wasDpadUpHeld && !isDpadUpHeld;
        isDpadDownReleased = wasDpadDownHeld && !isDpadDownHeld;
        isDpadLeftReleased = wasDpadLeftHeld && !isDpadLeftHeld;
        isDpadRightReleased = wasDpadRightHeld && !isDpadRightHeld;
        isLeftBumperReleased = wasLeftBumperHeld && !isLeftBumperHeld;
        isRightBumperReleased = wasRightBumperHeld && !isRightBumperHeld;
        isLeftStickReleased = wasLeftStickHeld && !isLeftStickHeld;
        isRightStickReleased = wasRightStickHeld && !isRightStickHeld;
        isBackReleased = wasBackHeld && !isBackHeld;
        isStartReleased = wasStartHeld && !isStartHeld;

    }

    public boolean isAHeld() {
        return isAHeld;
    }

    public boolean isAPressed() {
        return isAPressed;
    }

    public boolean isAReleased() {
        return isAReleased;
    }

    public boolean isBHeld() {
        return isBHeld;
    }

    public boolean isBPressed() {
        return isBPressed;
    }

    public boolean isBReleased() {
        return isBReleased;
    }

    public boolean isXHeld() {
        return isXHeld;
    }

    public boolean isXPressed() {
        return isXPressed;
    }

    public boolean isXReleased() {
        return isXReleased;
    }

    public boolean isYHeld() {
        return isYHeld;
    }

    public boolean isYPressed() {
        return isYPressed;
    }

    public boolean isYReleased() {
        return isYReleased;
    }

    public boolean isDpadUpHeld() {
        return isDpadUpHeld;
    }

    public boolean isDpadUpPressed() {
        return isDpadUpPressed;
    }

    public boolean isDpadUpReleased() {
        return isDpadUpReleased;
    }

    public boolean isDpadDownHeld() {
        return isDpadDownHeld;
    }

    public boolean isDpadDownPressed() {
        return isDpadDownPressed;
    }

    public boolean isDpadDownReleased() {
        return isDpadDownReleased;
    }

    public boolean isDpadLeftHeld() {
        return isDpadLeftHeld;
    }

    public boolean isDpadLeftPressed() {
        return isDpadLeftPressed;
    }

    public boolean isDpadLeftReleased() {
        return isDpadLeftReleased;
    }

    public boolean isDpadRightHeld() {
        return isDpadRightHeld;
    }

    public boolean isDpadRightPressed() {
        return isDpadRightPressed;
    }

    public boolean isDpadRightReleased() {
        return isDpadRightReleased;
    }

    public boolean isLeftBumperHeld() {
        return isLeftBumperHeld;
    }

    public boolean isLeftBumperPressed() {
        return isLeftBumperPressed;
    }

    public boolean isLeftBumperReleased() {
        return isLeftBumperReleased;
    }

    public boolean isRightBumperHeld() {
        return isRightBumperHeld;
    }

    public boolean isRightBumperPressed() {
        return isRightBumperPressed;
    }

    public boolean isRightBumperReleased() {
        return isRightBumperReleased;
    }

    public boolean isLeftStickHeld() {
        return isLeftStickHeld;
    }

    public boolean isLeftStickPressed() {
        return isLeftStickPressed;
    }

    public boolean isLeftStickReleased() {
        return isLeftStickReleased;
    }

    public boolean isRightStickHeld() {
        return isRightStickHeld;
    }

    public boolean isRightStickPressed() {
        return isRightStickPressed;
    }

    public boolean isRightStickReleased() {
        return isRightStickReleased;
    }

    public boolean isBackHeld() {
        return isBackHeld;
    }

    public boolean isBackPressed() {
        return isBackPressed;
    }

    public boolean isBackReleased() {
        return isBackReleased;
    }

    public boolean isStartHeld() {
        return isStartHeld;
    }

    public boolean isStartPressed() {
        return isStartPressed;
    }

    public boolean isStartReleased() {
        return isStartReleased;
    }

}
