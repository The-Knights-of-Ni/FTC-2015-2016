/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Util;

public abstract class LinearOpMode
extends OpMode {
    private a a = null;
    private Thread b = null;
    private ElapsedTime c = new ElapsedTime();
    private volatile boolean d = false;

    public abstract void runOpMode() throws InterruptedException;

    public synchronized void waitForStart() throws InterruptedException {
        while (!this.d) {
            LinearOpMode linearOpMode = this;
            synchronized (linearOpMode) {
                this.wait();
                continue;
            }
        }
    }

    public void waitOneFullHardwareCycle() throws InterruptedException {
        this.waitForNextHardwareCycle();
        Thread.sleep(1);
        this.waitForNextHardwareCycle();
    }

    public void waitForNextHardwareCycle() throws InterruptedException {
        LinearOpMode linearOpMode = this;
        synchronized (linearOpMode) {
            this.wait();
        }
    }

    public void sleep(long milliseconds) throws InterruptedException {
        Thread.sleep(milliseconds);
    }

    public boolean opModeIsActive() {
        return this.d;
    }

    @Override
    public final void init() {
        this.a = new a(this);
        this.b = new Thread((Runnable)this.a, "Linear OpMode Helper");
        this.b.start();
    }

    @Override
    public final void init_loop() {
        this.a();
    }

    @Override
    public final void start() {
        this.d = true;
        LinearOpMode linearOpMode = this;
        synchronized (linearOpMode) {
            this.notifyAll();
        }
    }

    @Override
    public final void loop() {
        this.a();
    }

    @Override
    public final void stop() {
        this.d = false;
        if (!this.a.c()) {
            this.b.interrupt();
        }
        this.c.reset();
        while (!(this.a.c() || this.c.time() >= 0.5)) {
            Thread.yield();
        }
        if (!this.a.c()) {
            RobotLog.e("*****************************************************************");
            RobotLog.e("User Linear Op Mode took too long to exit; emergency killing app.");
            RobotLog.e("Possible infinite loop in user code?");
            RobotLog.e("*****************************************************************");
            System.exit(-1);
        }
    }

    private void a() {
        if (this.a.a()) {
            throw this.a.b();
        }
        LinearOpMode linearOpMode = this;
        synchronized (linearOpMode) {
            this.notifyAll();
        }
    }

    private static class a
    implements Runnable {
        private RuntimeException a = null;
        private boolean b = false;
        private final LinearOpMode c;

        public a(LinearOpMode linearOpMode) {
            this.c = linearOpMode;
        }

        @Override
        public void run() {
            Util.logThreadLifeCycle("LinearOpModeHelper.run()", new Runnable(){

                @Override
                public void run() {
                    a.this.a = null;
                    a.this.b = false;
                    try {
                        a.this.c.runOpMode();
                    }
                    catch (InterruptedException var1_1) {
                        RobotLog.d("LinearOpMode received an Interrupted Exception; shutting down this linear op mode");
                    }
                    catch (RuntimeException var1_2) {
                        a.this.a = var1_2;
                    }
                    finally {
                        a.this.b = true;
                    }
                }
            });
        }

        public boolean a() {
            return this.a != null;
        }

        public RuntimeException b() {
            return this.a;
        }

        public boolean c() {
            return this.b;
        }

    }

}

