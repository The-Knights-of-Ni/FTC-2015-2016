/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
port com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.Telemetry;
import java.util.concurrent.TimeUnit;

public abstract class OpMode {
    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();
    public Telemetry telemetry = new Telemetry();
    public HardwareMap hardwareMap = new HardwareMap();
    public double time = 0.0;
    private long a = System.nanoTime();

    public abstract void init();

    public void init_loop() {
    }

    public void start() {
    }

    public abstract void loop();

    public void stop() {
    }

    public double getRuntime() {
        double d = TimeUnit.SECONDS.toNanos(1);
        return (double)(System.nanoTime() - this.a) / d;
    }

    public void resetStartTime() {
        this.a = System.nanoTime();
    }
}

