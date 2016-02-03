/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  com.qualcomm.hardware.HardwareFactory
 *  com.qualcomm.modernrobotics.ModernRoboticsUsbUtil
 *  com.qualcomm.robotcore.eventloop.EventLoop
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.eventloop.opmode.OpMode
 *  com.qualcomm.robotcore.eventloop.opmode.OpModeManager
 *  com.qualcomm.robotcore.eventloop.opmode.OpModeRegister
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.Gamepad
 *  com.qualcomm.robotcore.hardware.HardwareMap
 *  com.qualcomm.robotcore.robocol.Command
 *  com.qualcomm.robotcore.robocol.Telemetry
 *  com.qualcomm.robotcore.util.Util
 */
package com.qualcomm.ftccommon;

import android.content.Context;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.modernrobotics.ModernRoboticsUsbUtil;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Util;
import java.util.Set;

public class FtcEventLoop
implements EventLoop {
    FtcEventLoopHandler a;
    OpModeManager b = new OpModeManager(new HardwareMap());
    OpModeRegister c;

    public FtcEventLoop(HardwareFactory hardwareFactory, OpModeRegister register, UpdateUI.Callback callback, Context robotControllerContext) {
        this.a = new FtcEventLoopHandler(hardwareFactory, callback, robotControllerContext);
        this.c = register;
    }

    public OpModeManager getOpModeManager() {
        return this.b;
    }

    public synchronized void init(EventLoopManager eventLoopManager) throws RobotCoreException, InterruptedException {
        DbgLog.msg("======= INIT START =======");
        this.b.registerOpModes(this.c);
        this.a.init(eventLoopManager);
        HardwareMap hardwareMap = this.a.getHardwareMap();
        ModernRoboticsUsbUtil.init((Context)hardwareMap.appContext, (HardwareMap)hardwareMap);
        this.b.setHardwareMap(hardwareMap);
        hardwareMap.logDevices();
        DbgLog.msg("======= INIT FINISH =======");
    }

    public synchronized void loop() throws RobotCoreException {
        this.a.displayGamePadInfo(this.b.getActiveOpModeName());
        Gamepad[] arrgamepad = this.a.getGamepads();
        this.b.runActiveOpMode(arrgamepad);
        this.a.sendTelemetryData(this.b.getActiveOpMode().telemetry);
    }

    public synchronized void teardown() throws RobotCoreException {
        DbgLog.msg("======= TEARDOWN =======");
        this.b.stopActiveOpMode();
        this.a.shutdownMotorControllers();
        this.a.shutdownServoControllers();
        this.a.shutdownLegacyModules();
        this.a.shutdownCoreInterfaceDeviceModules();
        DbgLog.msg("======= TEARDOWN COMPLETE =======");
    }

    public synchronized void processCommand(Command command) {
        DbgLog.msg("Processing Command: %s(%d) %s", command.getName(), command.getSequenceNumber(), command.getExtra());
        this.a.sendBatteryInfo();
        String string = command.getName();
        String string2 = command.getExtra();
        if (string.equals("CMD_RESTART_ROBOT")) {
            this.a();
        } else if (string.equals("CMD_REQUEST_OP_MODE_LIST")) {
            this.b();
        } else if (string.equals("CMD_INIT_OP_MODE")) {
            this.a(string2);
        } else if (string.equals("CMD_RUN_OP_MODE")) {
            this.c();
        } else {
            DbgLog.msg("Unknown command: " + string);
        }
    }

    private void a() {
        this.a.restartRobot();
    }

    private void b() {
        String string = "";
        for (String string2 : this.b.getOpModes()) {
            if (string2.equals("Stop Robot")) continue;
            if (!string.isEmpty()) {
                string = string + Util.ASCII_RECORD_SEPARATOR;
            }
            string = string + string2;
        }
        this.a.sendCommand(new Command("CMD_REQUEST_OP_MODE_LIST_RESP", string));
    }

    private void a(String string) {
        String string2 = this.a.getOpMode(string);
        this.b.initActiveOpMode(string2);
        this.a.sendCommand(new Command("CMD_INIT_OP_MODE_RESP", string2));
    }

    private void c() {
        this.b.startActiveOpMode();
        this.a.sendCommand(new Command("CMD_RUN_OP_MODE_RESP", this.b.getActiveOpModeName()));
    }
}

