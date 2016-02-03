/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.eventloop;

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.robocol.Heartbeat;
import com.qualcomm.robotcore.robocol.PeerDiscovery;
import com.qualcomm.robotcore.robocol.RobocolDatagram;
import com.qualcomm.robotcore.robocol.RobocolDatagramSocket;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Util;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class EventLoopManager {
    public static final String SYSTEM_TELEMETRY = "SYSTEM_TELEMETRY";
    public static final String ROBOT_BATTERY_LEVEL_KEY = "Robot Battery Level";
    public static final String RC_BATTERY_LEVEL_KEY = "RobotController Battery Level";
    private static final EventLoop a = new a();
    public RobotState state = RobotState.NOT_STARTED;
    private Thread b = new Thread();
    private ExecutorService c = Executors.newFixedThreadPool(2);
    private final RobocolDatagramSocket d;
    private ElapsedTime e = new ElapsedTime();
    private EventLoop f = a;
    private final Gamepad[] g = new Gamepad[]{new Gamepad(), new Gamepad()};
    private Heartbeat h = new Heartbeat(Heartbeat.Token.EMPTY);
    private EventLoopMonitor i = null;
    private final Set<SyncdDevice> j = new CopyOnWriteArraySet<SyncdDevice>();
    private final Command[] k = new Command[8];
    private int l = 0;
    private final Set<Command> m = new CopyOnWriteArraySet<Command>();
    private InetAddress n;

    public void handleDroppedConnection() {
        OpModeManager opModeManager = this.f.getOpModeManager();
        String string = "Lost connection while running op mode: " + opModeManager.getActiveOpModeName();
        opModeManager.initActiveOpMode("Stop Robot");
        this.a(RobotState.DROPPED_CONNECTION);
        RobotLog.i(string);
    }

    public EventLoopManager(RobocolDatagramSocket socket) {
        this.d = socket;
        this.a(RobotState.NOT_STARTED);
    }

    public void setMonitor(EventLoopMonitor monitor) {
        this.i = monitor;
    }

    public EventLoopMonitor getMonitor() {
        return this.i;
    }

    public void start(EventLoop eventLoop) throws RobotCoreException {
        this.c = Executors.newFixedThreadPool(2);
        this.c.submit(new d());
        this.c.submit(new c());
        this.setEventLoop(eventLoop);
    }

    public void shutdown() {
        this.d.close();
        this.c.shutdownNow();
        try {
            this.c.awaitTermination(30, TimeUnit.DAYS);
        }
        catch (InterruptedException var1_1) {
            // empty catch block
        }
        this.b();
    }

    public void registerSyncdDevice(SyncdDevice device) {
        this.j.add(device);
    }

    public void unregisterSyncdDevice(SyncdDevice device) {
        this.j.remove(device);
    }

    public void setEventLoop(EventLoop eventLoop) throws RobotCoreException {
        if (eventLoop == null) {
            eventLoop = a;
            RobotLog.d("Event loop cannot be null, using empty event loop");
        }
        this.b();
        this.f = eventLoop;
        this.a();
    }

    public EventLoop getEventLoop() {
        return this.f;
    }

    public Gamepad getGamepad() {
        return this.getGamepad(0);
    }

    public Gamepad getGamepad(int port) {
        Range.throwIfRangeIsInvalid(port, 0.0, 1.0);
        return this.g[port];
    }

    public Gamepad[] getGamepads() {
        return this.g;
    }

    public Heartbeat getHeartbeat() {
        return this.h;
    }

    public void sendTelemetryData(Telemetry telemetry) {
        try {
            this.d.send(new RobocolDatagram(telemetry.toByteArrayForTransmission()));
        }
        catch (RobotCoreException var2_2) {
            RobotLog.w("Failed to send telemetry data");
            RobotLog.logStacktrace(var2_2);
        }
        telemetry.clearData();
    }

    public void sendCommand(Command command) {
        this.m.add(command);
    }

    private void a() throws RobotCoreException {
        try {
            this.a(RobotState.INIT);
            this.f.init(this);
            for (SyncdDevice syncdDevice : this.j) {
                syncdDevice.startBlockingWork();
            }
        }
        catch (Exception var1_2) {
            RobotLog.w("Caught exception during looper init: " + var1_2.toString());
            RobotLog.logStacktrace(var1_2);
            this.a(RobotState.EMERGENCY_STOP);
            if (RobotLog.hasGlobalErrorMsg()) {
                this.buildAndSendTelemetry("SYSTEM_TELEMETRY", RobotLog.getGlobalErrorMsg());
            }
            throw new RobotCoreException("Robot failed to start: " + var1_2.getMessage());
        }
        this.e = new ElapsedTime(0);
        this.a(RobotState.RUNNING);
        this.b = new Thread((Runnable)new b(), "Event Loop");
        this.b.start();
    }

    private void b() {
        this.b.interrupt();
        try {
            Thread.sleep(200);
        }
        catch (InterruptedException var1_1) {
            // empty catch block
        }
        this.a(RobotState.STOPPED);
        this.f = a;
        this.j.clear();
    }

    private void a(RobotState robotState) {
        this.state = robotState;
        RobotLog.v("EventLoopManager state is " + robotState.toString());
        if (this.i != null) {
            this.i.onStateChange(robotState);
        }
    }

    private void a(RobocolDatagram robocolDatagram) throws RobotCoreException {
        Gamepad gamepad = new Gamepad();
        gamepad.fromByteArray(robocolDatagram.getData());
        if (gamepad.user < 1 || gamepad.user > 2) {
            RobotLog.d("Gamepad with user %d received. Only users 1 and 2 are valid");
            return;
        }
        int n = gamepad.user - 1;
        this.g[n].copy(gamepad);
        if (this.g[0].id == this.g[1].id) {
            RobotLog.v("Gamepad moved position, removing stale gamepad");
            if (n == 0) {
                this.g[1].copy(new Gamepad());
            }
            if (n == 1) {
                this.g[0].copy(new Gamepad());
            }
        }
    }

    private void b(RobocolDatagram robocolDatagram) throws RobotCoreException {
        Heartbeat heartbeat = new Heartbeat(Heartbeat.Token.EMPTY);
        heartbeat.fromByteArray(robocolDatagram.getData());
        heartbeat.setRobotState(this.state);
        robocolDatagram.setData(heartbeat.toByteArrayForTransmission());
        this.d.send(robocolDatagram);
        this.e.reset();
        this.h = heartbeat;
    }

    private void c(RobocolDatagram robocolDatagram) throws RobotCoreException {
        if (robocolDatagram.getAddress().equals(this.n)) {
            return;
        }
        PeerDiscovery peerDiscovery = PeerDiscovery.forReceive();
        peerDiscovery.fromByteArray(robocolDatagram.getData());
        if (this.state == RobotState.DROPPED_CONNECTION) {
            this.a(RobotState.RUNNING);
        }
        if (this.f == a) {
            return;
        }
        this.n = robocolDatagram.getAddress();
        RobotLog.i("new remote peer discovered: " + this.n.getHostAddress());
        try {
            this.d.connect(this.n);
        }
        catch (SocketException var3_3) {
            RobotLog.e("Unable to connect to peer:" + var3_3.toString());
        }
        PeerDiscovery peerDiscovery2 = new PeerDiscovery(PeerDiscovery.PeerType.PEER);
        RobotLog.v("sending peer discovery packet(%d)", peerDiscovery2.getSequenceNumber());
        RobocolDatagram robocolDatagram2 = new RobocolDatagram(peerDiscovery2);
        if (this.d.getInetAddress() == null) {
            robocolDatagram2.setAddress(this.n);
        }
        this.d.send(robocolDatagram2);
    }

    private void d(RobocolDatagram robocolDatagram) throws RobotCoreException {
        Command command = new Command(robocolDatagram.getData());
        if (command.isAcknowledged()) {
            this.m.remove(command);
            return;
        }
        command.acknowledge();
        this.d.send(new RobocolDatagram(command));
        for (Command command2 : this.k) {
            if (command2 == null || !command2.equals(command)) continue;
            return;
        }
        this.k[this.l++ % this.k.length] = command;
        try {
            this.f.processCommand(command);
        }
        catch (Exception var3_4) {
            RobotLog.e("Event loop threw an exception while processing a command");
            RobotLog.logStacktrace(var3_4);
        }
    }

    private void c() {
    }

    private void e(RobocolDatagram robocolDatagram) {
        RobotLog.w("RobotCore event loop received unknown event type: " + robocolDatagram.getMsgType().name());
    }

    public void buildAndSendTelemetry(String tag, String msg) {
        Telemetry telemetry = new Telemetry();
        telemetry.setTag(tag);
        telemetry.addData(tag, msg);
        this.sendTelemetryData(telemetry);
    }

    static /* synthetic */ void a(EventLoopManager eventLoopManager, RobocolDatagram robocolDatagram) throws RobotCoreException {
        eventLoopManager.a(robocolDatagram);
    }

    static /* synthetic */ void b(EventLoopManager eventLoopManager, RobocolDatagram robocolDatagram) throws RobotCoreException {
        eventLoopManager.b(robocolDatagram);
    }

    static /* synthetic */ void c(EventLoopManager eventLoopManager, RobocolDatagram robocolDatagram) throws RobotCoreException {
        eventLoopManager.c(robocolDatagram);
    }

    static /* synthetic */ void d(EventLoopManager eventLoopManager, RobocolDatagram robocolDatagram) throws RobotCoreException {
        eventLoopManager.d(robocolDatagram);
    }

    static /* synthetic */ void c(EventLoopManager eventLoopManager) {
        eventLoopManager.c();
    }

    static /* synthetic */ void e(EventLoopManager eventLoopManager, RobocolDatagram robocolDatagram) {
        eventLoopManager.e(robocolDatagram);
    }

    public static enum State {
        NOT_STARTED,
        INIT,
        RUNNING,
        STOPPED,
        EMERGENCY_STOP,
        DROPPED_CONNECTION;
        

        private State() {
        }
    }

    private static class a
    implements EventLoop {
        private a() {
        }

        @Override
        public void init(EventLoopManager eventProcessor) {
        }

        @Override
        public void loop() {
        }

        @Override
        public void teardown() {
        }

        @Override
        public void processCommand(Command command) {
            RobotLog.w("Dropping command " + command.getName() + ", no active event loop");
        }

        @Override
        public OpModeManager getOpModeManager() {
            return null;
        }
    }

    private class b
    implements Runnable {
        private b() {
        }

        @Override
        public void run() {
            Util.logThreadLifeCycle("EventLoopRunnable.run()", new Runnable(){

                @Override
                public void run() {
                    block18 : {
                        try {
                            ElapsedTime elapsedTime = new ElapsedTime();
                            double d = 0.001;
                            long l = 5;
                            while (!Thread.interrupted()) {
                                while (elapsedTime.time() < 0.001) {
                                    Thread.sleep(5);
                                }
                                elapsedTime.reset();
                                if (RobotLog.hasGlobalErrorMsg()) {
                                    EventLoopManager.this.buildAndSendTelemetry("SYSTEM_TELEMETRY", RobotLog.getGlobalErrorMsg());
                                }
                                if (EventLoopManager.this.e.startTime() == 0.0) {
                                    Thread.sleep(500);
                                } else if (EventLoopManager.this.e.time() > 2.0) {
                                    EventLoopManager.this.handleDroppedConnection();
                                    EventLoopManager.this.n = null;
                                    EventLoopManager.this.e = new ElapsedTime(0);
                                }
                                for (Object object2 : EventLoopManager.this.j) {
                                    object2.blockUntilReady();
                                }
                                try {
                                    EventLoopManager.this.f.loop();
                                    continue;
                                }
                                catch (Exception var6_8) {
                                    Object object2;
                                    RobotLog.e("Event loop threw an exception");
                                    RobotLog.logStacktrace(var6_8);
                                    object2 = var6_8.getClass().getSimpleName() + (var6_8.getMessage() != null ? new StringBuilder().append(" - ").append(var6_8.getMessage()).toString() : "");
                                    RobotLog.setGlobalErrorMsg("User code threw an uncaught exception: " + (String)object2);
                                    EventLoopManager.this.buildAndSendTelemetry("SYSTEM_TELEMETRY", RobotLog.getGlobalErrorMsg());
                                    throw new RobotCoreException("EventLoop Exception in loop()");
                                }
                                finally {
                                    for (SyncdDevice syncdDevice : EventLoopManager.this.j) {
                                        syncdDevice.startBlockingWork();
                                    }
                                    continue;
                                }
                            }
                        }
                        catch (InterruptedException var1_2) {
                            RobotLog.v("EventLoopRunnable interrupted");
                            EventLoopManager.this.a(RobotState.STOPPED);
                        }
                        catch (RobotCoreException var1_3) {
                            RobotLog.v("RobotCoreException in EventLoopManager: " + var1_3.getMessage());
                            EventLoopManager.this.a(RobotState.EMERGENCY_STOP);
                            EventLoopManager.this.buildAndSendTelemetry("SYSTEM_TELEMETRY", RobotLog.getGlobalErrorMsg());
                        }
                        try {
                            EventLoopManager.this.f.teardown();
                        }
                        catch (Exception var1_4) {
                            RobotLog.w("Caught exception during looper teardown: " + var1_4.toString());
                            RobotLog.logStacktrace(var1_4);
                            if (!RobotLog.hasGlobalErrorMsg()) break block18;
                            EventLoopManager.this.buildAndSendTelemetry("SYSTEM_TELEMETRY", RobotLog.getGlobalErrorMsg());
                        }
                    }
                    RobotLog.v("EventLoopRunnable has exited");
                }
            });
        }

    }

    private class c
    implements Runnable {
        ElapsedTime a;

        private c() {
            this.a = new ElapsedTime();
        }

        @Override
        public void run() {
            Util.logThreadLifeCycle("RecvRunnable.run()", new Runnable(){

                /*
                 * Unable to fully structure code
                 * Enabled force condition propagation
                 * Lifted jumps to return sites
                 */
                @Override
                public void run() {
lbl1: // 10 sources:
                    block12 : while (!Thread.interrupted()) {
                        var1_1 = EventLoopManager.b(c.this.EventLoopManager.this).recv();
                        if (Thread.interrupted() || EventLoopManager.b(c.this.EventLoopManager.this).isClosed()) {
                            return;
                        }
                        if (var1_1 == null) {
                            Thread.yield();
                            continue;
                        }
                        if (RobotLog.hasGlobalErrorMsg()) {
                            c.this.EventLoopManager.this.buildAndSendTelemetry("SYSTEM_TELEMETRY", RobotLog.getGlobalErrorMsg());
                        }
                        try {
                            switch (1.a[var1_1.getMsgType().ordinal()]) {
                                case 1: {
                                    EventLoopManager.a(c.this.EventLoopManager.this, var1_1);
                                    ** break;
                                }
                                case 2: {
                                    EventLoopManager.b(c.this.EventLoopManager.this, var1_1);
                                    ** break;
                                }
                                case 3: {
                                    EventLoopManager.c(c.this.EventLoopManager.this, var1_1);
                                    ** break;
                                }
                                case 4: {
                                    EventLoopManager.d(c.this.EventLoopManager.this, var1_1);
                                    ** break;
                                }
                                case 5: {
                                    EventLoopManager.c(c.this.EventLoopManager.this);
                                    ** break;
                                }
                                default: {
                                    EventLoopManager.e(c.this.EventLoopManager.this, var1_1);
                                    continue block12;
                                }
                            }
                        }
                        catch (RobotCoreException var2_2) {
                            RobotLog.w("RobotCore event loop cannot process event: " + var2_2.toString());
                            RobotLog.setGlobalErrorMsg(var2_2.getMessage());
                            continue;
                        }
                        finally {
                            var1_1.close();
                            continue;
                        }
                    }
                }
            });
        }

    }

    private class d
    implements Runnable {
        private Set<Command> b;

        private d() {
            this.b = new HashSet<Command>();
        }

        @Override
        public void run() {
            Util.logThreadLifeCycle("ScheduledSendRunnable.run()", new Runnable(){

                @Override
                public void run() {
                    while (!Thread.interrupted()) {
                        long l = System.nanoTime();
                        for (Command command : EventLoopManager.this.m) {
                            if (command.getAttempts() > 10) {
                                RobotLog.w("giving up on command %s(%d) after %d attempts", command.getName(), command.getSequenceNumber(), Byte.valueOf(command.getAttempts()));
                                d.this.b.add(command);
                                continue;
                            }
                            if (command.isAcknowledged()) {
                                RobotLog.v("command %s(%d) has been acknowledged by remote device", command.getName(), command.getSequenceNumber());
                                d.this.b.add(command);
                                continue;
                            }
                            if (!command.shouldTransmit(l)) continue;
                            try {
                                RobotLog.v("sending %s(%d), attempt %d", command.getName(), command.getSequenceNumber(), Byte.valueOf(command.getAttempts()));
                                EventLoopManager.this.d.send(new RobocolDatagram(command.toByteArrayForTransmission()));
                            }
                            catch (RobotCoreException var5_5) {
                                RobotLog.w("failed to send %s(%d) ", command.getName(), command.getSequenceNumber());
                                RobotLog.logStacktrace(var5_5);
                            }
                        }
                        EventLoopManager.this.m.removeAll(d.this.b);
                        d.this.b.clear();
                        try {
                            Thread.sleep(100);
                            continue;
                        }
                        catch (InterruptedException var3_3) {
                            return;
                        }
                    }
                }
            });
        }

    }

    public static interface EventLoopMonitor {
        public void onStateChange(RobotState var1);
    }

}

