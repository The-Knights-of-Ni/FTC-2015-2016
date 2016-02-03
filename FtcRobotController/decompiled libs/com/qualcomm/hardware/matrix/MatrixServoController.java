/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.ServoController
 *  com.qualcomm.robotcore.hardware.ServoController$PwmStatus
 *  com.qualcomm.robotcore.util.Range
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.matrix;

import com.qualcomm.hardware.matrix.MatrixI2cTransaction;
import com.qualcomm.hardware.matrix.MatrixMasterController;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.Arrays;

public class MatrixServoController
implements ServoController {
    public static final int SERVO_POSITION_MAX = 240;
    private MatrixMasterController a;
    protected ServoController.PwmStatus pwmStatus;
    protected double[] servoCache = new double[4];

    public MatrixServoController(MatrixMasterController master) {
        this.a = master;
        this.pwmStatus = ServoController.PwmStatus.DISABLED;
        Arrays.fill(this.servoCache, 0.0);
        master.registerServoController(this);
    }

    public void pwmEnable() {
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction(0, MatrixI2cTransaction.a.h, 15);
        this.a.queueTransaction(matrixI2cTransaction);
        this.pwmStatus = ServoController.PwmStatus.ENABLED;
    }

    public void pwmDisable() {
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction(0, MatrixI2cTransaction.a.h, 0);
        this.a.queueTransaction(matrixI2cTransaction);
        this.pwmStatus = ServoController.PwmStatus.DISABLED;
    }

    public ServoController.PwmStatus getPwmStatus() {
        return this.pwmStatus;
    }

    public void setServoPosition(int channel, double position) {
        this.a(channel);
        Range.throwIfRangeIsInvalid((double)position, (double)0.0, (double)1.0);
        byte by = (byte)(position * 240.0);
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)channel, by, 0);
        this.a.queueTransaction(matrixI2cTransaction);
    }

    public void setServoPosition(int channel, double position, byte speed) {
        this.a(channel);
        Range.throwIfRangeIsInvalid((double)position, (double)0.0, (double)1.0);
        byte by = (byte)(position * 240.0);
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)channel, by, speed);
        this.a.queueTransaction(matrixI2cTransaction);
    }

    public double getServoPosition(int channel) {
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)channel, MatrixI2cTransaction.a.g);
        if (this.a.queueTransaction(matrixI2cTransaction)) {
            this.a.waitOnRead();
        }
        return this.servoCache[channel] / 240.0;
    }

    public String getDeviceName() {
        return "Matrix Servo Controller";
    }

    public String getConnectionInfo() {
        return this.a.getConnectionInfo();
    }

    public int getVersion() {
        return 1;
    }

    public void close() {
        this.pwmDisable();
    }

    private void a(int n) {
        if (n < 1 || n > 4) {
            throw new IllegalArgumentException(String.format("Channel %d is invalid; valid channels are 1..%d", n, Byte.valueOf(4)));
        }
    }

    public void handleReadServo(MatrixI2cTransaction transaction, byte[] buffer) {
        this.servoCache[transaction.servo] = TypeConversion.unsignedByteToInt((byte)buffer[4]);
    }
}

