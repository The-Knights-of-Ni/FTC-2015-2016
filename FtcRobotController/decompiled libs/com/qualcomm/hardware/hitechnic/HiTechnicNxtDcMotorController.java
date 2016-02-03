/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.DcMotorController
 *  com.qualcomm.robotcore.hardware.DcMotorController$DeviceMode
 *  com.qualcomm.robotcore.hardware.DcMotorController$RunMode
 *  com.qualcomm.robotcore.hardware.I2cController
 *  com.qualcomm.robotcore.hardware.I2cController$I2cPortReadyCallback
 *  com.qualcomm.robotcore.util.ElapsedTime
 *  com.qualcomm.robotcore.util.Range
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.hitechnic;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.concurrent.locks.Lock;

public class HiTechnicNxtDcMotorController
implements DcMotorController,
I2cController.I2cPortReadyCallback {
    public static final int MIN_MOTOR = 1;
    public static final int MAX_MOTOR = 2;
    public static final byte POWER_MAX = 100;
    public static final byte POWER_BREAK = 0;
    public static final byte POWER_MIN = -100;
    public static final byte POWER_FLOAT = -128;
    public static final int I2C_ADDRESS = 2;
    public static final int MEM_START_ADDRESS = 64;
    public static final int MEM_READ_LENGTH = 20;
    public static final int OFFSET_MOTOR1_TARGET_ENCODER_VALUE = 4;
    public static final int OFFSET_MOTOR1_MODE = 8;
    public static final int OFFSET_MOTOR1_POWER = 9;
    public static final int OFFSET_MOTOR2_POWER = 10;
    public static final int OFFSET_MOTOR2_MODE = 11;
    public static final int OFFSET_MOTOR2_TARGET_ENCODER_VALUE = 12;
    public static final int OFFSET_MOTOR1_CURRENT_ENCODER_VALUE = 16;
    public static final int OFFSET_MOTOR2_CURRENT_ENCODER_VALUE = 20;
    public static final int OFFSET_UNUSED = -1;
    public static final int NUM_BYTES = 20;
    public static final int CHANNEL_MODE_MASK_SELECTION = 3;
    public static final int CHANNEL_MODE_MASK_LOCK = 4;
    public static final int CHANNEL_MODE_MASK_REVERSE = 8;
    public static final int CHANNEL_MODE_MASK_NO_TIMEOUT = 16;
    public static final int CHANNEL_MODE_MASK_EMPTY_D5 = 32;
    public static final int CHANNEL_MODE_MASK_ERROR = 64;
    public static final int CHANNEL_MODE_MASK_BUSY = 128;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RUN_POWER_CONTROL_ONLY_NXT = 0;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RUN_CONSTANT_SPEED_NXT = 1;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RUN_TO_POSITION = 2;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RESET = 3;
    public static final byte[] OFFSET_MAP_MOTOR_POWER = new byte[]{-1, 9, 10};
    public static final byte[] OFFSET_MAP_MOTOR_MODE = new byte[]{-1, 8, 11};
    public static final byte[] OFFSET_MAP_MOTOR_TARGET_ENCODER_VALUE = new byte[]{-1, 4, 12};
    public static final byte[] OFFSET_MAP_MOTOR_CURRENT_ENCODER_VALUE = new byte[]{-1, 16, 20};
    private final ModernRoboticsUsbLegacyModule a;
    private final byte[] b;
    private final Lock c;
    private final byte[] d;
    private final Lock e;
    private final int f;
    private final ElapsedTime g = new ElapsedTime(0);
    private volatile DcMotorController.DeviceMode h;
    private volatile boolean i = true;

    public HiTechnicNxtDcMotorController(ModernRoboticsUsbLegacyModule legacyModule, int physicalPort) {
        this.a = legacyModule;
        this.f = physicalPort;
        this.b = legacyModule.getI2cReadCache(physicalPort);
        this.c = legacyModule.getI2cReadCacheLock(physicalPort);
        this.d = legacyModule.getI2cWriteCache(physicalPort);
        this.e = legacyModule.getI2cWriteCacheLock(physicalPort);
        this.h = DcMotorController.DeviceMode.WRITE_ONLY;
        legacyModule.enableI2cWriteMode(physicalPort, 2, 64, 20);
        try {
            this.e.lock();
            this.d[9] = -128;
            this.d[10] = -128;
        }
        finally {
            this.e.unlock();
        }
        legacyModule.writeI2cCacheToController(physicalPort);
        legacyModule.registerForI2cPortReadyCallback(this, physicalPort);
    }

    public String getDeviceName() {
        return "NXT DC Motor Controller";
    }

    public String getConnectionInfo() {
        return this.a.getConnectionInfo() + "; port " + this.f;
    }

    public int getVersion() {
        return 1;
    }

    public void setMotorControllerDeviceMode(DcMotorController.DeviceMode mode) {
        if (this.h == mode) {
            return;
        }
        switch (mode) {
            case READ_ONLY: {
                this.h = DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE;
                this.a.enableI2cReadMode(this.f, 2, 64, 20);
                break;
            }
            case WRITE_ONLY: {
                this.h = DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE;
                this.a.enableI2cWriteMode(this.f, 2, 64, 20);
            }
        }
        this.i = true;
    }

    public DcMotorController.DeviceMode getMotorControllerDeviceMode() {
        return this.h;
    }

    public void setMotorChannelMode(int motor, DcMotorController.RunMode mode) {
        this.a(motor);
        this.a();
        byte by = HiTechnicNxtDcMotorController.runModeToFlagNXT(mode);
        try {
            this.e.lock();
            if (this.d[OFFSET_MAP_MOTOR_MODE[motor]] != by) {
                this.d[HiTechnicNxtDcMotorController.OFFSET_MAP_MOTOR_MODE[motor]] = by;
                this.i = true;
            }
        }
        finally {
            this.e.unlock();
        }
    }

    public DcMotorController.RunMode getMotorChannelMode(int motor) {
        this.a(motor);
        this.b();
        byte by = 0;
        try {
            this.c.lock();
            by = this.b[OFFSET_MAP_MOTOR_MODE[motor]];
        }
        finally {
            this.c.unlock();
        }
        return HiTechnicNxtDcMotorController.flagToRunModeNXT(by);
    }

    public void setMotorPower(int motor, double power) {
        this.a(motor);
        this.a();
        Range.throwIfRangeIsInvalid((double)power, (double)-1.0, (double)1.0);
        byte by = (byte)(power * 100.0);
        try {
            this.e.lock();
            if (by != this.d[OFFSET_MAP_MOTOR_POWER[motor]]) {
                this.d[HiTechnicNxtDcMotorController.OFFSET_MAP_MOTOR_POWER[motor]] = by;
                this.i = true;
            }
        }
        finally {
            this.e.unlock();
        }
    }

    public double getMotorPower(int motor) {
        this.a(motor);
        this.b();
        byte by = 0;
        try {
            this.c.lock();
            by = this.b[OFFSET_MAP_MOTOR_POWER[motor]];
        }
        finally {
            this.c.unlock();
        }
        double d = by == -128 ? 0.0 : (double)by / 100.0;
        return d;
    }

    public boolean isBusy(int motor) {
        this.a(motor);
        this.b();
        boolean bl = false;
        try {
            this.c.lock();
            bl = (128 & this.b[OFFSET_MAP_MOTOR_MODE[motor]]) == 128;
        }
        finally {
            this.c.unlock();
        }
        return bl;
    }

    public void setMotorPowerFloat(int motor) {
        this.a(motor);
        this.a();
        try {
            this.e.lock();
            if (-128 != this.d[OFFSET_MAP_MOTOR_POWER[motor]]) {
                this.d[HiTechnicNxtDcMotorController.OFFSET_MAP_MOTOR_POWER[motor]] = -128;
                this.i = true;
            }
        }
        finally {
            this.e.unlock();
        }
    }

    public boolean getMotorPowerFloat(int motor) {
        this.a(motor);
        this.b();
        boolean bl = false;
        try {
            this.c.lock();
            bl = this.b[OFFSET_MAP_MOTOR_POWER[motor]] == -128;
        }
        finally {
            this.c.unlock();
        }
        return bl;
    }

    public void setMotorTargetPosition(int motor, int position) {
        this.a(motor);
        this.a();
        byte[] arrby = TypeConversion.intToByteArray((int)position);
        try {
            this.e.lock();
            System.arraycopy(arrby, 0, this.d, OFFSET_MAP_MOTOR_TARGET_ENCODER_VALUE[motor], arrby.length);
            this.i = true;
        }
        finally {
            this.e.unlock();
        }
    }

    public int getMotorTargetPosition(int motor) {
        this.a(motor);
        this.b();
        byte[] arrby = new byte[4];
        try {
            this.c.lock();
            System.arraycopy(this.b, OFFSET_MAP_MOTOR_TARGET_ENCODER_VALUE[motor], arrby, 0, arrby.length);
        }
        finally {
            this.c.unlock();
        }
        return TypeConversion.byteArrayToInt((byte[])arrby);
    }

    public int getMotorCurrentPosition(int motor) {
        this.a(motor);
        this.b();
        byte[] arrby = new byte[4];
        try {
            this.c.lock();
            System.arraycopy(this.b, OFFSET_MAP_MOTOR_CURRENT_ENCODER_VALUE[motor], arrby, 0, arrby.length);
        }
        finally {
            this.c.unlock();
        }
        return TypeConversion.byteArrayToInt((byte[])arrby);
    }

    public void close() {
        if (this.h == DcMotorController.DeviceMode.WRITE_ONLY) {
            this.setMotorPowerFloat(1);
            this.setMotorPowerFloat(2);
        }
    }

    private void a() {
        if (this.h == DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
            return;
        }
        if (this.h == DcMotorController.DeviceMode.READ_ONLY || this.h == DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
            String string = "Cannot write while in this mode: " + (Object)this.h;
            StackTraceElement[] arrstackTraceElement = Thread.currentThread().getStackTrace();
            if (arrstackTraceElement != null && arrstackTraceElement.length > 3) {
                string = string + "\n from method: " + arrstackTraceElement[3].getMethodName();
            }
            throw new IllegalArgumentException(string);
        }
    }

    private void b() {
        if (this.h == DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
            return;
        }
        if (this.h == DcMotorController.DeviceMode.WRITE_ONLY || this.h == DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
            String string = "Cannot read while in this mode: " + (Object)this.h;
            StackTraceElement[] arrstackTraceElement = Thread.currentThread().getStackTrace();
            if (arrstackTraceElement != null && arrstackTraceElement.length > 3) {
                string = string + "\n from method: " + arrstackTraceElement[3].getMethodName();
            }
            throw new IllegalArgumentException(string);
        }
    }

    private void a(int n) {
        if (n < 1 || n > 2) {
            throw new IllegalArgumentException(String.format("Motor %d is invalid; valid motors are 1..%d", n, 2));
        }
    }

    public static DcMotorController.RunMode flagToRunModeNXT(byte flag) {
        switch (flag & 3) {
            case 0: {
                return DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
            }
            case 1: {
                return DcMotorController.RunMode.RUN_USING_ENCODERS;
            }
            case 2: {
                return DcMotorController.RunMode.RUN_TO_POSITION;
            }
            case 3: {
                return DcMotorController.RunMode.RESET_ENCODERS;
            }
        }
        return DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
    }

    public static byte runModeToFlagNXT(DcMotorController.RunMode mode) {
        switch (mode) {
            case RUN_USING_ENCODERS: {
                return 1;
            }
            case RUN_WITHOUT_ENCODERS: {
                return 0;
            }
            case RUN_TO_POSITION: {
                return 2;
            }
            case RESET_ENCODERS: {
                return 3;
            }
        }
        return 1;
    }

    public void portIsReady(int port) {
        switch (this.h) {
            case SWITCHING_TO_READ_MODE: {
                if (!this.a.isI2cPortInReadMode(port)) break;
                this.h = DcMotorController.DeviceMode.READ_ONLY;
                break;
            }
            case SWITCHING_TO_WRITE_MODE: {
                if (!this.a.isI2cPortInWriteMode(port)) break;
                this.h = DcMotorController.DeviceMode.WRITE_ONLY;
            }
        }
        if (this.h == DcMotorController.DeviceMode.READ_ONLY) {
            this.a.setI2cPortActionFlag(this.f);
            this.a.writeI2cPortFlagOnlyToController(this.f);
        } else {
            if (this.i || this.g.time() > 2.0) {
                this.a.setI2cPortActionFlag(this.f);
                this.a.writeI2cCacheToController(this.f);
                this.g.reset();
            }
            this.i = false;
        }
        this.a.readI2cCacheFromController(this.f);
    }

}

