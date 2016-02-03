/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.I2cController
 *  com.qualcomm.robotcore.hardware.I2cController$I2cPortReadyCallback
 *  com.qualcomm.robotcore.util.ElapsedTime
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.matrix;

import com.qualcomm.hardware.matrix.MatrixDcMotorController;
import com.qualcomm.hardware.matrix.MatrixI2cTransaction;
import com.qualcomm.hardware.matrix.MatrixServoController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;

public class MatrixMasterController
implements I2cController.I2cPortReadyCallback {
    private static final byte[] a = new byte[]{0, 70, 72, 74, 76};
    private static final byte[] b = new byte[]{0, 78, 88, 98, 108};
    private static final byte[] c = new byte[]{0, 82, 92, 102, 112};
    private static final byte[] d = new byte[]{0, 86, 96, 106, 116};
    private static final byte[] e = new byte[]{0, 87, 97, 107, 117};
    protected ConcurrentLinkedQueue<MatrixI2cTransaction> transactionQueue;
    protected ModernRoboticsUsbLegacyModule legacyModule;
    protected MatrixDcMotorController motorController;
    protected MatrixServoController servoController;
    protected int physicalPort;
    private volatile boolean f = false;
    private final ElapsedTime g = new ElapsedTime(0);

    public MatrixMasterController(ModernRoboticsUsbLegacyModule legacyModule, int physicalPort) {
        this.legacyModule = legacyModule;
        this.physicalPort = physicalPort;
        this.transactionQueue = new ConcurrentLinkedQueue();
        legacyModule.registerForI2cPortReadyCallback(this, physicalPort);
    }

    public void registerMotorController(MatrixDcMotorController mc) {
        this.motorController = mc;
    }

    public void registerServoController(MatrixServoController sc) {
        this.servoController = sc;
    }

    public int getPort() {
        return this.physicalPort;
    }

    public String getConnectionInfo() {
        return this.legacyModule.getConnectionInfo() + "; port " + this.physicalPort;
    }

    public boolean queueTransaction(MatrixI2cTransaction transaction, boolean force) {
        if (!force) {
            for (MatrixI2cTransaction matrixI2cTransaction : this.transactionQueue) {
                if (!matrixI2cTransaction.isEqual(transaction)) continue;
                this.buginf("NO Queue transaction " + transaction.toString());
                return false;
            }
        }
        this.buginf("YES Queue transaction " + transaction.toString());
        this.transactionQueue.add(transaction);
        return true;
    }

    public boolean queueTransaction(MatrixI2cTransaction transaction) {
        return this.queueTransaction(transaction, false);
    }

    public void waitOnRead() {
        MatrixMasterController matrixMasterController = this;
        synchronized (matrixMasterController) {
            this.f = true;
            try {
                while (this.f) {
                    this.wait(0);
                }
            }
            catch (InterruptedException var2_2) {
                var2_2.printStackTrace();
            }
        }
    }

    protected void handleReadDone(MatrixI2cTransaction transaction) {
        byte[] arrby = this.legacyModule.getI2cReadCache(this.physicalPort);
        switch (transaction.property) {
            case d: {
                this.motorController.handleReadBattery(arrby);
                break;
            }
            case e: {
                this.motorController.handleReadPosition(transaction, arrby);
                break;
            }
            case b: {
                this.motorController.handleReadPosition(transaction, arrby);
                break;
            }
            case a: {
                this.motorController.handleReadMode(transaction, arrby);
                break;
            }
            case g: {
                this.servoController.handleReadServo(transaction, arrby);
                break;
            }
            default: {
                RobotLog.e((String)("Transaction not a read " + (Object)transaction.property));
            }
        }
        MatrixMasterController matrixMasterController = this;
        synchronized (matrixMasterController) {
            if (this.f) {
                this.f = false;
                this.notify();
            }
        }
    }

    protected void sendHeartbeat() {
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction(0, MatrixI2cTransaction.a.j, 3);
        this.queueTransaction(matrixI2cTransaction);
    }

    public void portIsReady(int port) {
        byte[] arrby;
        int n;
        int n2;
        if (this.transactionQueue.isEmpty()) {
            if (this.g.time() > 2.0) {
                this.sendHeartbeat();
                this.g.reset();
            }
            return;
        }
        MatrixI2cTransaction matrixI2cTransaction = this.transactionQueue.peek();
        if (matrixI2cTransaction.state == MatrixI2cTransaction.b.b) {
            this.legacyModule.readI2cCacheFromModule(this.physicalPort);
            matrixI2cTransaction.state = MatrixI2cTransaction.b.d;
            return;
        }
        if (matrixI2cTransaction.state == MatrixI2cTransaction.b.c) {
            matrixI2cTransaction = this.transactionQueue.poll();
            if (this.transactionQueue.isEmpty()) {
                return;
            }
            matrixI2cTransaction = this.transactionQueue.peek();
        } else if (matrixI2cTransaction.state == MatrixI2cTransaction.b.d) {
            this.handleReadDone(matrixI2cTransaction);
            matrixI2cTransaction = this.transactionQueue.poll();
            if (this.transactionQueue.isEmpty()) {
                return;
            }
            matrixI2cTransaction = this.transactionQueue.peek();
        }
        switch (matrixI2cTransaction.property) {
            case e: {
                n2 = b[matrixI2cTransaction.motor];
                n = 4;
                arrby = new byte[]{0};
                break;
            }
            case d: {
                n2 = 67;
                arrby = new byte[]{0};
                n = 1;
                break;
            }
            case j: {
                n2 = 66;
                arrby = new byte[]{(byte)matrixI2cTransaction.value};
                n = 1;
                break;
            }
            case i: {
                n2 = 68;
                arrby = new byte[]{(byte)matrixI2cTransaction.value};
                n = 1;
                break;
            }
            case c: {
                n2 = d[matrixI2cTransaction.motor];
                arrby = new byte[]{(byte)matrixI2cTransaction.value};
                n = 1;
                break;
            }
            case b: {
                n2 = c[matrixI2cTransaction.motor];
                arrby = TypeConversion.intToByteArray((int)matrixI2cTransaction.value);
                n = 4;
                break;
            }
            case a: {
                n2 = e[matrixI2cTransaction.motor];
                arrby = new byte[]{(byte)matrixI2cTransaction.value};
                n = 1;
                break;
            }
            case f: {
                n2 = b[matrixI2cTransaction.motor];
                ByteBuffer byteBuffer = ByteBuffer.allocate(10);
                byteBuffer.put(TypeConversion.intToByteArray((int)0));
                byteBuffer.put(TypeConversion.intToByteArray((int)matrixI2cTransaction.target));
                byteBuffer.put(matrixI2cTransaction.speed);
                byteBuffer.put(matrixI2cTransaction.mode);
                arrby = byteBuffer.array();
                n = 10;
                break;
            }
            case g: {
                n2 = a[matrixI2cTransaction.servo];
                arrby = new byte[]{matrixI2cTransaction.speed, (byte)matrixI2cTransaction.target};
                n = 2;
                break;
            }
            case h: {
                n2 = 69;
                arrby = new byte[]{(byte)matrixI2cTransaction.value};
                n = 1;
                break;
            }
            default: {
                n2 = 0;
                arrby = new byte[]{(byte)matrixI2cTransaction.value};
                n = 1;
            }
        }
        try {
            if (matrixI2cTransaction.write) {
                this.legacyModule.setWriteMode(this.physicalPort, 16, n2);
                this.legacyModule.setData(this.physicalPort, arrby, n);
                matrixI2cTransaction.state = MatrixI2cTransaction.b.c;
            } else {
                this.legacyModule.setReadMode(this.physicalPort, 16, n2, n);
                matrixI2cTransaction.state = MatrixI2cTransaction.b.b;
            }
            this.legacyModule.setI2cPortActionFlag(this.physicalPort);
            this.legacyModule.writeI2cCacheToModule(this.physicalPort);
        }
        catch (IllegalArgumentException var6_7) {
            RobotLog.e((String)var6_7.getMessage());
        }
        this.buginf(matrixI2cTransaction.toString());
    }

    protected void buginf(String s) {
    }

}

