/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public class OpModeManager {
    public static final String DEFAULT_OP_MODE_NAME = "Stop Robot";
    public static final OpMode DEFAULT_OP_MODE = new a();
    private Map<String, Class<?>> a = new LinkedHashMap();
    private Map<String, OpMode> b = new LinkedHashMap<String, OpMode>();
    private String c = "Stop Robot";
    private OpMode d = DEFAULT_OP_MODE;
    private String e = "Stop Robot";
    private HardwareMap f = new HardwareMap();
    private HardwareMap g = new HardwareMap();
    private b h = b.a;
    private boolean i = false;
    private boolean j = false;
    private boolean k = false;
    private boolean l = false;

    public OpModeManager(HardwareMap hardwareMap) {
        this.f = hardwareMap;
        this.register("Stop Robot", a.class);
        this.initActiveOpMode("Stop Robot");
    }

    public synchronized void registerOpModes(OpModeRegister register) {
        register.register(this);
    }

    public synchronized void setHardwareMap(HardwareMap hardwareMap) {
        this.f = hardwareMap;
    }

    public synchronized HardwareMap getHardwareMap() {
        return this.f;
    }

    public synchronized Set<String> getOpModes() {
        LinkedHashSet<String> linkedHashSet = new LinkedHashSet<String>();
        linkedHashSet.addAll(this.a.keySet());
        linkedHashSet.addAll(this.b.keySet());
        return linkedHashSet;
    }

    public synchronized String getActiveOpModeName() {
        return this.c;
    }

    public synchronized OpMode getActiveOpMode() {
        return this.d;
    }

    public synchronized void initActiveOpMode(String name) {
        this.e = name;
        this.i = true;
        this.j = true;
        this.l = true;
        this.h = b.a;
    }

    public synchronized void startActiveOpMode() {
        this.h = b.b;
        this.k = true;
    }

    public synchronized void stopActiveOpMode() {
        this.d.stop();
        this.initActiveOpMode("Stop Robot");
    }

    public synchronized void runActiveOpMode(Gamepad[] gamepads) {
        this.d.time = this.d.getRuntime();
        this.d.gamepad1 = gamepads[0];
        this.d.gamepad2 = gamepads[1];
        if (this.l) {
            this.d.gamepad1.reset();
            this.d.gamepad2.reset();
            this.l = false;
        }
        if (this.i) {
            this.d.stop();
            this.a();
            this.h = b.a;
            this.j = true;
        }
        if (this.h == b.a) {
            if (this.j) {
                this.d.hardwareMap = this.f;
                this.d.resetStartTime();
                this.d.init();
                this.j = false;
            }
            this.d.init_loop();
        } else {
            if (this.k) {
                this.d.start();
                this.k = false;
            }
            this.d.loop();
        }
    }

    public synchronized void logOpModes() {
        int n = this.a.size() + this.b.size();
        RobotLog.i("There are " + n + " Op Modes");
        for (Map.Entry entry2 : this.a.entrySet()) {
            RobotLog.i("   Op Mode: " + entry2.getKey());
        }
        for (Map.Entry entry : this.b.entrySet()) {
            RobotLog.i("   Op Mode: " + (String)entry.getKey());
        }
    }

    public synchronized void register(String name, Class opMode) {
        if (this.a(name)) {
            throw new IllegalArgumentException("Cannot register the same op mode name twice");
        }
        this.a.put(name, opMode);
    }

    public synchronized void register(String name, OpMode opMode) {
        if (this.a(name)) {
            throw new IllegalArgumentException("Cannot register the same op mode name twice");
        }
        this.b.put(name, opMode);
    }

    private void a() {
        RobotLog.i("Attempting to switch to op mode " + this.e);
        try {
            this.d = this.b.containsKey(this.e) ? this.b.get(this.e) : (OpMode)this.a.get(this.e).newInstance();
            this.c = this.e;
        }
        catch (InstantiationException var1_1) {
            this.a(var1_1);
        }
        catch (IllegalAccessException var1_2) {
            this.a(var1_2);
        }
        this.i = false;
    }

    private boolean a(String string) {
        return this.getOpModes().contains(string);
    }

    private void a(Exception exception) {
        RobotLog.e("Unable to start op mode " + this.c);
        RobotLog.logStacktrace(exception);
        this.c = "Stop Robot";
        this.d = DEFAULT_OP_MODE;
    }

    private static class a
    extends OpMode {
        @Override
        public void init() {
            this.a();
        }

        @Override
        public void init_loop() {
            this.a();
            this.telemetry.addData("Status", "Robot is stopped");
        }

        @Override
        public void loop() {
            this.a();
            this.telemetry.addData("Status", "Robot is stopped");
        }

        @Override
        public void stop() {
        }

        private void a() {
            for (ServoController servoController222 : this.hardwareMap.servoController) {
                servoController222.pwmDisable();
            }
            for (DcMotorController dcMotorController : this.hardwareMap.dcMotorController) {
                dcMotorController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
            }
            for (DcMotor dcMotor : this.hardwareMap.dcMotor) {
                dcMotor.setPower(0.0);
                dcMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }
            for (LightSensor lightSensor : this.hardwareMap.lightSensor) {
                lightSensor.enableLed(false);
            }
        }
    }

    private static enum b {
        a,
        b;
        

        private b() {
        }
    }

}

