/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  com.qualcomm.hardware.HardwareFactory
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.DcMotorController
 *  com.qualcomm.robotcore.hardware.DeviceInterfaceModule
 *  com.qualcomm.robotcore.hardware.Gamepad
 *  com.qualcomm.robotcore.hardware.HardwareMap
 *  com.qualcomm.robotcore.hardware.HardwareMap$DeviceMapping
 *  com.qualcomm.robotcore.hardware.LegacyModule
 *  com.qualcomm.robotcore.hardware.ServoController
 *  com.qualcomm.robotcore.robocol.Command
 *  com.qualcomm.robotcore.robocol.Telemetry
 *  com.qualcomm.robotcore.robot.RobotState
 *  com.qualcomm.robotcore.util.BatteryChecker
 *  com.qualcomm.robotcore.util.BatteryChecker$BatteryWatcher
 *  com.qualcomm.robotcore.util.ElapsedTime
 */
package com.qualcomm.ftccommon;

import android.content.Context;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Map;
import java.util.Set;

public class FtcEventLoopHandler
implements BatteryChecker.BatteryWatcher {
    public static final String NO_VOLTAGE_SENSOR = "no voltage sensor found";
    EventLoopManager a;
    BatteryChecker b;
    Context c;
    ElapsedTime d = new ElapsedTime();
    double e = 0.25;
    UpdateUI.Callback f;
    HardwareFactory g;
    HardwareMap h = new HardwareMap();

    public FtcEventLoopHandler(HardwareFactory hardwareFactory, UpdateUI.Callback callback, Context robotControllerContext) {
        this.g = hardwareFactory;
        this.f = callback;
        this.c = robotControllerContext;
        long l = 180000;
        this.b = new BatteryChecker(robotControllerContext, (BatteryChecker.BatteryWatcher)this, l);
        this.b.startBatteryMonitoring();
    }

    public void init(EventLoopManager eventLoopManager) {
        this.a = eventLoopManager;
    }

    public HardwareMap getHardwareMap() throws RobotCoreException, InterruptedException {
        this.h = this.g.createHardwareMap(this.a);
        return this.h;
    }

    public void displayGamePadInfo(String activeOpModeName) {
        Gamepad[] arrgamepad = this.a.getGamepads();
        this.f.updateUi(activeOpModeName, arrgamepad);
    }

    public Gamepad[] getGamepads() {
        return this.a.getGamepads();
    }

    public void sendTelemetryData(Telemetry telemetry) {
        if (this.d.time() > this.e) {
            this.d.reset();
            if (telemetry.hasData()) {
                this.a.sendTelemetryData(telemetry);
            }
            telemetry.clearData();
        }
    }

    public void sendBatteryInfo() {
        this.a();
        this.b();
    }

    private void a() {
        float f = this.b.getBatteryLevel();
        this.sendTelemetry("RobotController Battery Level", String.valueOf(f));
    }

    private void b() {
        Object object;
        double d = 1.7976931348623157E308;
        for (Object object22 : this.h.voltageSensor) {
            if (object22.getVoltage() >= d) continue;
            d = object22.getVoltage();
        }
        if (this.h.voltageSensor.size() == 0) {
            object = "no voltage sensor found";
        } else {
            Object object22;
            object22 = new BigDecimal(d).setScale(2, RoundingMode.HALF_UP);
            object = String.valueOf(object22.doubleValue());
        }
        this.sendTelemetry("Robot Battery Level", (String)object);
    }

    public void sendTelemetry(String tag, String msg) {
        Telemetry telemetry = new Telemetry();
        telemetry.setTag(tag);
        telemetry.addData(tag, msg);
        if (this.a != null) {
            this.a.sendTelemetryData(telemetry);
            telemetry.clearData();
        }
    }

    public void shutdownMotorControllers() {
        for (Map.Entry entry : this.h.dcMotorController.entrySet()) {
            String string = (String)entry.getKey();
            DcMotorController dcMotorController = (DcMotorController)entry.getValue();
            DbgLog.msg("Stopping DC Motor Controller " + string);
            dcMotorController.close();
        }
    }

    public void shutdownServoControllers() {
        for (Map.Entry entry : this.h.servoController.entrySet()) {
            String string = (String)entry.getKey();
            ServoController servoController = (ServoController)entry.getValue();
            DbgLog.msg("Stopping Servo Controller " + string);
            servoController.close();
        }
    }

    public void shutdownLegacyModules() {
        for (Map.Entry entry : this.h.legacyModule.entrySet()) {
            String string = (String)entry.getKey();
            LegacyModule legacyModule = (LegacyModule)entry.getValue();
            DbgLog.msg("Stopping Legacy Module " + string);
            legacyModule.close();
        }
    }

    public void shutdownCoreInterfaceDeviceModules() {
        for (Map.Entry entry : this.h.deviceInterfaceModule.entrySet()) {
            String string = (String)entry.getKey();
            DeviceInterfaceModule deviceInterfaceModule = (DeviceInterfaceModule)entry.getValue();
            DbgLog.msg("Stopping Core Interface Device Module " + string);
            deviceInterfaceModule.close();
        }
    }

    public void restartRobot() {
        this.b.endBatteryMonitoring();
        this.f.restartRobot();
    }

    public void sendCommand(Command command) {
        this.a.sendCommand(command);
    }

    public String getOpMode(String extra) {
        if (this.a.state != RobotState.RUNNING) {
            return "Stop Robot";
        }
        return extra;
    }

    public void updateBatteryLevel(float percent) {
        this.sendTelemetry("RobotController Battery Level", String.valueOf(percent));
    }
}

