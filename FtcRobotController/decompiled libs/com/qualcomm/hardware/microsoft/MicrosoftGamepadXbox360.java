/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.Gamepad
 *  com.qualcomm.robotcore.hardware.Gamepad$GamepadCallback
 */
package com.qualcomm.hardware.microsoft;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MicrosoftGamepadXbox360
extends Gamepad {
    public MicrosoftGamepadXbox360() {
        this(null);
    }

    public MicrosoftGamepadXbox360(Gamepad.GamepadCallback callback) {
        super(callback);
        this.joystickDeadzone = 0.15f;
    }

    public String type() {
        return "Xbox 360";
    }
}

