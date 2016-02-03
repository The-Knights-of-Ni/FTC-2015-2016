/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.DcMotor
 *  com.qualcomm.robotcore.hardware.DcMotor$Direction
 *  com.qualcomm.robotcore.hardware.DcMotorController
 *  com.qualcomm.robotcore.hardware.DcMotorController$DeviceMode
 *  com.qualcomm.robotcore.hardware.DcMotorController$RunMode
 *  com.qualcomm.robotcore.util.Range
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.matrix;

import com.qualcomm.hardware.matrix.MatrixI2cTransaction;
import com.qualcomm.hardware.matrix.MatrixMasterController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.Arrays;
import java.util.Set;

public class MatrixDcMotorController
implements DcMotorController {
    private a[] a = new a[]{new a(), new a(), new a(), new a(), new a()};
    public static final byte POWER_MAX = 100;
    public static final byte POWER_MIN = -100;
    protected MatrixMasterController master;
    protected DcMotorController.DeviceMode deviceMode;
    private int b;

    public MatrixDcMotorController(MatrixMasterController master) {
        this.master = master;
        this.b = 0;
        master.registerMotorController(this);
        for (byte by = 0; by < 4; by = (byte)(by + 1)) {
            MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction(by, 0, 0, 0);
            master.queueTransaction(matrixI2cTransaction);
            this.a[by].f = DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
            this.a[by].d = true;
        }
        this.deviceMode = DcMotorController.DeviceMode.READ_ONLY;
    }

    protected byte runModeToFlagMatrix(DcMotorController.RunMode mode) {
        switch (mode) {
            case RUN_USING_ENCODERS: {
                return 2;
            }
            case RUN_WITHOUT_ENCODERS: {
                return 1;
            }
            case RUN_TO_POSITION: {
                return 3;
            }
            case RESET_ENCODERS: {
                return 4;
            }
        }
        return 4;
    }

    protected DcMotorController.RunMode flagMatrixToRunMode(byte flag) {
        switch (flag) {
            case 2: {
                return DcMotorController.RunMode.RUN_USING_ENCODERS;
            }
            case 1: {
                return DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
            }
            case 3: {
                return DcMotorController.RunMode.RUN_TO_POSITION;
            }
            case 4: {
                return DcMotorController.RunMode.RESET_ENCODERS;
            }
        }
        RobotLog.e((String)("Invalid run mode flag " + flag));
        return DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
    }

    public boolean isBusy(int motor) {
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)motor, MatrixI2cTransaction.a.a);
        this.master.queueTransaction(matrixI2cTransaction);
        this.master.waitOnRead();
        if ((this.a[matrixI2cTransaction.motor].c & 128) != 0) {
            return true;
        }
        return false;
    }

    public void setMotorControllerDeviceMode(DcMotorController.DeviceMode mode) {
        this.deviceMode = mode;
    }

    public DcMotorController.DeviceMode getMotorControllerDeviceMode() {
        return this.deviceMode;
    }

    public void setMotorChannelMode(int motor, DcMotorController.RunMode mode) {
        this.a(motor);
        if (!(this.a[motor].d || this.a[motor].f != mode)) {
            return;
        }
        byte by = this.runModeToFlagMatrix(mode);
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)motor, MatrixI2cTransaction.a.a, (int)by);
        this.master.queueTransaction(matrixI2cTransaction);
        this.a[motor].f = mode;
        this.a[motor].d = mode == DcMotorController.RunMode.RESET_ENCODERS;
    }

    public DcMotorController.RunMode getMotorChannelMode(int motor) {
        this.a(motor);
        return this.a[motor].f;
    }

    public void setMotorPowerFloat(int motor) {
        this.a(motor);
        if (!this.a[motor].d) {
            MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)motor, MatrixI2cTransaction.a.a, 4);
            this.master.queueTransaction(matrixI2cTransaction);
        }
        this.a[motor].d = true;
    }

    public boolean getMotorPowerFloat(int motor) {
        this.a(motor);
        return this.a[motor].d;
    }

    public void setMotorPower(Set<DcMotor> motors, double power) {
        MatrixI2cTransaction matrixI2cTransaction;
        Range.throwIfRangeIsInvalid((double)power, (double)-1.0, (double)1.0);
        for (DcMotor dcMotor : motors) {
            byte by = (byte)(power * 100.0);
            if (dcMotor.getDirection() == DcMotor.Direction.REVERSE) {
                by = (byte)(by * -1);
            }
            int n = dcMotor.getPortNumber();
            matrixI2cTransaction = new MatrixI2cTransaction((byte)n, by, this.a[n].a, (byte)(this.runModeToFlagMatrix(this.a[n].f) | 8));
            this.master.queueTransaction(matrixI2cTransaction);
        }
        matrixI2cTransaction = new MatrixI2cTransaction(0, MatrixI2cTransaction.a.i, 1);
        this.master.queueTransaction(matrixI2cTransaction);
    }

    public void setMotorPower(int motor, double power) {
        this.a(motor);
        Range.throwIfRangeIsInvalid((double)power, (double)-1.0, (double)1.0);
        byte by = (byte)(power * 100.0);
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)motor, by, this.a[motor].a, this.runModeToFlagMatrix(this.a[motor].f));
        this.master.queueTransaction(matrixI2cTransaction);
        this.a[motor].e = power;
    }

    public double getMotorPower(int motor) {
        this.a(motor);
        return this.a[motor].e;
    }

    public void setMotorTargetPosition(int motor, int position) {
        this.a(motor);
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)motor, MatrixI2cTransaction.a.b, position);
        this.master.queueTransaction(matrixI2cTransaction);
        this.a[motor].a = position;
    }

    public int getMotorTargetPosition(int motor) {
        this.a(motor);
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)motor, MatrixI2cTransaction.a.b);
        if (this.master.queueTransaction(matrixI2cTransaction)) {
            this.master.waitOnRead();
        }
        return this.a[motor].a;
    }

    public int getMotorCurrentPosition(int motor) {
        this.a(motor);
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction((byte)motor, MatrixI2cTransaction.a.e);
        if (this.master.queueTransaction(matrixI2cTransaction)) {
            this.master.waitOnRead();
        }
        return this.a[motor].b;
    }

    public int getBattery() {
        MatrixI2cTransaction matrixI2cTransaction = new MatrixI2cTransaction(0, MatrixI2cTransaction.a.d);
        if (this.master.queueTransaction(matrixI2cTransaction)) {
            this.master.waitOnRead();
        }
        return this.b;
    }

    public String getDeviceName() {
        return "Matrix Motor Controller";
    }

    public String getConnectionInfo() {
        return this.master.getConnectionInfo();
    }

    public int getVersion() {
        return 1;
    }

    public void close() {
        this.setMotorPowerFloat(1);
        this.setMotorPowerFloat(2);
        this.setMotorPowerFloat(3);
        this.setMotorPowerFloat(4);
    }

    public void handleReadBattery(byte[] buffer) {
        this.b = 40 * TypeConversion.unsignedByteToInt((byte)buffer[4]);
        RobotLog.v((String)("Battery voltage: " + this.b + "mV"));
    }

    public void handleReadPosition(MatrixI2cTransaction transaction, byte[] buffer) {
        this.a[transaction.motor].b = TypeConversion.byteArrayToInt((byte[])Arrays.copyOfRange(buffer, 4, 8));
        RobotLog.v((String)("Position motor: " + transaction.motor + " " + this.a[transaction.motor].b));
    }

    public void handleReadTargetPosition(MatrixI2cTransaction transaction, byte[] buffer) {
        this.a[transaction.motor].a = TypeConversion.byteArrayToInt((byte[])Arrays.copyOfRange(buffer, 4, 8));
        RobotLog.v((String)("Target motor: " + transaction.motor + " " + this.a[transaction.motor].a));
    }

    public void handleReadMode(MatrixI2cTransaction transaction, byte[] buffer) {
        this.a[transaction.motor].c = buffer[4];
        RobotLog.v((String)("Mode: " + this.a[transaction.motor].c));
    }

    private void a(int n) {
        if (n < 1 || n > 4) {
            throw new IllegalArgumentException(String.format("Motor %d is invalid; valid motors are 1..%d", n, 4));
        }
    }

    private class a {
        public int a;
        public int b;
        public byte c;
        public boolean d;
        public double e;
        public DcMotorController.RunMode f;

        public a() {
            this.a = 0;
            this.b = 0;
            this.c = 0;
            this.e = 0.0;
            this.d = true;
            this.f = DcMotorController.RunMode.RESET_ENCODERS;
        }
    }

}

