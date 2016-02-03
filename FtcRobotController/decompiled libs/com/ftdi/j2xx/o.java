/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  android.content.Intent
 *  android.support.v4.content.LocalBroadcastManager
 *  android.util.Log
 */
package com.ftdi.j2xx;

import android.content.Context;
import android.content.Intent;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Log;
import com.ftdi.j2xx.D2xxManager;
import com.ftdi.j2xx.FT_Device;
import com.ftdi.j2xx.n;
import com.ftdi.j2xx.q;
import java.io.IOException;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.channels.Pipe;
import java.nio.channels.SelectableChannel;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

class o {
    private Semaphore[] a;
    private Semaphore[] b;
    private n[] c;
    private ByteBuffer d;
    private ByteBuffer[] e;
    private Pipe f;
    private Pipe.SinkChannel g;
    private Pipe.SourceChannel h;
    private int i;
    private int j;
    private Object k;
    private FT_Device l;
    private D2xxManager.DriverParameters m;
    private Lock n;
    private Condition o;
    private boolean p;
    private Lock q;
    private Condition r;
    private Object s;
    private int t;

    public o(FT_Device fT_Device) {
        this.l = fT_Device;
        this.m = this.l.d();
        this.i = this.m.getBufferNumber();
        int n = this.m.getMaxBufferSize();
        this.t = this.l.e();
        this.a = new Semaphore[this.i];
        this.b = new Semaphore[this.i];
        this.c = new n[this.i];
        this.e = new ByteBuffer[256];
        this.n = new ReentrantLock();
        this.o = this.n.newCondition();
        this.p = false;
        this.q = new ReentrantLock();
        this.r = this.q.newCondition();
        this.k = new Object();
        this.s = new Object();
        this.h();
        this.d = ByteBuffer.allocateDirect(n);
        try {
            this.f = Pipe.open();
            this.g = this.f.sink();
            this.h = this.f.source();
        }
        catch (IOException var3_3) {
            Log.d((String)"ProcessInCtrl", (String)"Create mMainPipe failed!");
            var3_3.printStackTrace();
        }
        for (int i = 0; i < this.i; ++i) {
            this.c[i] = new n(n);
            this.b[i] = new Semaphore(1);
            this.a[i] = new Semaphore(1);
            try {
                this.c(i);
                continue;
            }
            catch (Exception var4_5) {
                Log.d((String)"ProcessInCtrl", (String)("Acquire read buffer " + i + " failed!"));
                var4_5.printStackTrace();
            }
        }
    }

    boolean a() {
        return this.p;
    }

    D2xxManager.DriverParameters b() {
        return this.m;
    }

    n a(int n) {
        n n2 = null;
        n[] arrn = this.c;
        synchronized (arrn) {
            if (n >= 0 && n < this.i) {
                n2 = this.c[n];
            }
        }
        return n2;
    }

    n b(int n) throws InterruptedException {
        n n2 = null;
        this.a[n].acquire();
        n2 = this.a(n);
        if (n2.c(n) == null) {
            n2 = null;
        }
        return n2;
    }

    n c(int n) throws InterruptedException {
        n n2 = null;
        this.b[n].acquire();
        n2 = this.a(n);
        return n2;
    }

    public void d(int n) throws InterruptedException {
        n[] arrn = this.c;
        synchronized (arrn) {
            this.c[n].d(n);
        }
        this.a[n].release();
    }

    public void e(int n) throws InterruptedException {
        this.b[n].release();
    }

    public void a(n n) throws D2xxManager.D2xxException {
        int n2 = 0;
        short s = 0;
        short s2 = 0;
        boolean bl = false;
        try {
            int n3;
            int n4;
            n2 = n.b();
            if (n2 < 2) {
                n.a().clear();
                return;
            }
            Object object = this.s;
            synchronized (object) {
                n3 = this.d();
                n4 = n2 - 2;
                if (n3 < n4) {
                    Log.d((String)"ProcessBulkIn::", (String)" Buffer is full, waiting for read....");
                    this.a(bl, s, s2);
                    this.n.lock();
                    this.p = true;
                }
            }
            if (n3 < n4) {
                this.o.await();
                this.n.unlock();
            }
            this.b(n);
        }
        catch (InterruptedException var6_8) {
            this.n.unlock();
            Log.e((String)"ProcessInCtrl", (String)"Exception in Full await!");
            var6_8.printStackTrace();
        }
        catch (Exception var6_9) {
            Log.e((String)"ProcessInCtrl", (String)"Exception in ProcessBulkIN");
            var6_9.printStackTrace();
            throw new D2xxManager.D2xxException("Fatal error in BulkIn.");
        }
    }

    private void b(n n) throws InterruptedException {
        int n2 = 0;
        int n3 = 0;
        int n4 = 0;
        int n5 = 0;
        int n6 = 0;
        long l = 0;
        short s = 0;
        short s2 = 0;
        boolean bl = false;
        ByteBuffer byteBuffer = null;
        byteBuffer = n.a();
        n2 = n.b();
        if (n2 > 0) {
            n3 = n2 / this.t + (n2 % this.t > 0 ? 1 : 0);
            for (int i = 0; i < n3; ++i) {
                if (i == n3 - 1) {
                    n6 = n2;
                    byteBuffer.limit(n6);
                    n5 = i * this.t;
                    byteBuffer.position(n5);
                    byte by = byteBuffer.get();
                    s = (short)(this.l.g.modemStatus ^ (short)(by & 240));
                    this.l.g.modemStatus = (short)(by & 240);
                    byte by2 = byteBuffer.get();
                    this.l.g.lineStatus = (short)(by2 & 255);
                    n5+=2;
                    s2 = byteBuffer.hasRemaining() ? (short)(this.l.g.lineStatus & 30) : 0;
                } else {
                    n6 = (i + 1) * this.t;
                    byteBuffer.limit(n6);
                    n5 = i * this.t + 2;
                    byteBuffer.position(n5);
                }
                n4+=n6 - n5;
                this.e[i] = byteBuffer.slice();
            }
            if (n4 != 0) {
                bl = true;
                try {
                    l = this.g.write(this.e, 0, n3);
                    if (l != (long)n4) {
                        Log.d((String)"extractReadData::", (String)("written != totalData, written= " + l + " totalData=" + n4));
                    }
                    this.f((int)l);
                    this.q.lock();
                    this.r.signalAll();
                    this.q.unlock();
                }
                catch (Exception var13_13) {
                    Log.d((String)"extractReadData::", (String)"Write data to sink failed!!");
                    var13_13.printStackTrace();
                }
            }
            byteBuffer.clear();
            this.a(bl, s, s2);
        }
    }

    public int a(byte[] arrby, int n, long l) {
        boolean bl = false;
        int n2 = 0;
        int n3 = this.m.getMaxBufferSize();
        long l2 = System.currentTimeMillis();
        ByteBuffer byteBuffer = ByteBuffer.wrap(arrby, 0, n);
        if (l == 0) {
            l = this.m.getReadTimeout();
        }
        while (this.l.isOpen()) {
            if (this.c() >= n) {
                Object object = this.h;
                synchronized (object) {
                    try {
                        this.h.read(byteBuffer);
                        this.g(n);
                    }
                    catch (Exception var12_11) {
                        Log.d((String)"readBulkInData::", (String)"Cannot read data from Source!!");
                        var12_11.printStackTrace();
                    }
                }
                object = this.s;
                synchronized (object) {
                    if (this.p) {
                        Log.i((String)"FTDI debug::", (String)"buffer is full , and also re start buffer");
                        this.n.lock();
                        this.o.signalAll();
                        this.p = false;
                        this.n.unlock();
                    }
                }
                n2 = n;
                break;
            }
            try {
                this.q.lock();
                this.r.await(System.currentTimeMillis() - l2, TimeUnit.MILLISECONDS);
                this.q.unlock();
            }
            catch (InterruptedException var11_9) {
                Log.d((String)"readBulkInData::", (String)"Cannot wait to read data!!");
                var11_9.printStackTrace();
                this.q.unlock();
            }
            if (System.currentTimeMillis() - l2 >= l) break;
        }
        return n2;
    }

    private int f(int n) {
        int n2;
        Object object = this.k;
        synchronized (object) {
            this.j+=n;
            n2 = this.j;
        }
        return n2;
    }

    private int g(int n) {
        int n2;
        Object object = this.k;
        synchronized (object) {
            this.j-=n;
            n2 = this.j;
        }
        return n2;
    }

    private void h() {
        Object object = this.k;
        synchronized (object) {
            this.j = 0;
        }
    }

    public int c() {
        int n;
        Object object = this.k;
        synchronized (object) {
            n = this.j;
        }
        return n;
    }

    public int d() {
        return this.m.getMaxBufferSize() - this.c() - 1;
    }

    public int e() {
        int n = this.m.getBufferNumber();
        n n2 = null;
        int n3 = 0;
        ByteBuffer byteBuffer = this.d;
        synchronized (byteBuffer) {
            try {
                do {
                    this.h.configureBlocking(false);
                    n3 = this.h.read(this.d);
                    this.d.clear();
                } while (n3 != 0);
            }
            catch (Exception var5_5) {
                var5_5.printStackTrace();
            }
            this.h();
            for (int i = 0; i < n; ++i) {
                n2 = this.a(i);
                if (!n2.d() || n2.b() <= 2) continue;
                n2.c();
            }
        }
        return 0;
    }

    public int a(boolean bl, short s, short s2) throws InterruptedException {
        Intent intent;
        long l = 0;
        short s3 = 0;
        short s4 = 0;
        boolean bl2 = bl;
        s3 = s;
        s4 = s2;
        l = 0;
        q q = new q();
        q.a = this.l.i.a;
        if (bl2 && (q.a & 1) != 0 && (this.l.a ^ 1) == 1) {
            --this.l.a;
            intent = new Intent("FT_EVENT_RXCHAR");
            intent.putExtra("message", "FT_EVENT_RXCHAR");
            LocalBroadcastManager.getInstance((Context)this.l.j).sendBroadcast(intent);
        }
        if (s3 != 0 && (q.a & 2) != 0 && (this.l.a ^ 2) == 2) {
            this.l.a|=2;
            intent = new Intent("FT_EVENT_MODEM_STATUS");
            intent.putExtra("message", "FT_EVENT_MODEM_STATUS");
            LocalBroadcastManager.getInstance((Context)this.l.j).sendBroadcast(intent);
        }
        if (s4 != 0 && (q.a & 4) != 0 && (this.l.a ^ 4) == 4) {
            this.l.a|=4;
            intent = new Intent("FT_EVENT_LINE_STATUS");
            intent.putExtra("message", "FT_EVENT_LINE_STATUS");
            LocalBroadcastManager.getInstance((Context)this.l.j).sendBroadcast(intent);
        }
        return 0;
    }

    public void f() throws InterruptedException {
        int n = this.m.getBufferNumber();
        for (int i = 0; i < n; ++i) {
            if (!this.a(i).d()) continue;
            this.d(i);
        }
    }

    void g() {
        int n;
        for (n = 0; n < this.i; ++n) {
            try {
                this.e(n);
            }
            catch (Exception var2_2) {
                Log.d((String)"ProcessInCtrl", (String)("Acquire read buffer " + n + " failed!"));
                var2_2.printStackTrace();
            }
            this.c[n] = null;
            this.b[n] = null;
            this.a[n] = null;
        }
        for (n = 0; n < 256; ++n) {
            this.e[n] = null;
        }
        this.a = null;
        this.b = null;
        this.c = null;
        this.e = null;
        this.d = null;
        if (this.p) {
            this.n.lock();
            this.o.signalAll();
            this.n.unlock();
        }
        this.q.lock();
        this.r.signalAll();
        this.q.unlock();
        this.n = null;
        this.o = null;
        this.k = null;
        this.q = null;
        this.r = null;
        try {
            this.g.close();
            this.g = null;
            this.h.close();
            this.h = null;
            this.f = null;
        }
        catch (IOException var2_3) {
            Log.d((String)"ProcessInCtrl", (String)"Close mMainPipe failed!");
            var2_3.printStackTrace();
        }
        this.l = null;
        this.m = null;
    }
}

