/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.DcMotorController
 *  com.qualcomm.robotcore.hardware.DcMotorController$DeviceMode
 *  com.qualcomm.robotcore.hardware.DcMotorController$RunMode
 *  com.qualcomm.robotcore.hardware.VoltageSensor
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients
 *  com.qualcomm.robotcore.util.Range
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDevice;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnable;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnableBlocking;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;

public class ModernRoboticsUsbDcMotorController
extends ModernRoboticsUsbDevice
implements DcMotorController,
VoltageSensor {
    public static final boolean DEBUG_LOGGING = false;
    public static final int MONITOR_LENGTH = 30;
    public static final int MIN_MOTOR = 1;
    public static final int MAX_MOTOR = 2;
    public static final byte POWER_MAX = 100;
    public static final byte POWER_BREAK = 0;
    public static final byte POWER_MIN = -100;
    public static final byte POWER_FLOAT = -128;
    public static final byte RATIO_MIN = -128;
    public static final byte RATIO_MAX = 127;
    public static final int DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX = 255;
    public static final int BATTERY_MAX_MEASURABLE_VOLTAGE_INT = 1023;
    public static final double BATTERY_MAX_MEASURABLE_VOLTAGE = 20.4;
    public static final byte DEFAULT_P_COEFFICIENT = -128;
    public static final byte DEFAULT_I_COEFFICIENT = 64;
    public static final byte DEFAULT_D_COEFFICIENT = -72;
    public static final byte START_ADDRESS = 64;
    public static final int CHANNEL_MODE_MASK_SELECTION = 3;
    public static final int CHANNEL_MODE_MASK_LOCK = 4;
    public static final int CHANNEL_MODE_MASK_REVERSE = 8;
    public static final int CHANNEL_MODE_MASK_NO_TIMEOUT = 16;
    public static final int CHANNEL_MODE_MASK_EMPTY_D5 = 32;
    public static final int CHANNEL_MODE_MASK_ERROR = 64;
    public static final int CHANNEL_MODE_MASK_BUSY = 128;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RUN_POWER_CONTROL_ONLY = 0;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RUN_CONSTANT_SPEED = 1;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RUN_TO_POSITION = 2;
    public static final byte CHANNEL_MODE_FLAG_SELECT_RESET = 3;
    public static final byte CHANNEL_MODE_FLAG_LOCK = 4;
    public static final byte CHANNEL_MODE_FLAG_REVERSE = 8;
    public static final byte CHANNEL_MODE_FLAG_NO_TIMEOUT = 16;
    public static final byte CHANNEL_MODE_FLAG_UNUSED = 32;
    public static final byte CHANNEL_MODE_FLAG_ERROR = 64;
    public static final byte CHANNEL_MODE_FLAG_BUSY = -128;
    public static final int ADDRESS_MOTOR1_TARGET_ENCODER_VALUE = 64;
    public static final int ADDRESS_MOTOR1_MODE = 68;
    public static final int ADDRESS_MOTOR1_POWER = 69;
    public static final int ADDRESS_MOTOR2_POWER = 70;
    public static final int ADDRESS_MOTOR2_MODE = 71;
    public static final int ADDRESS_MOTOR2_TARGET_ENCODER_VALUE = 72;
    public static final int ADDRESS_MOTOR1_CURRENT_ENCODER_VALUE = 76;
    public static final int ADDRESS_MOTOR2_CURRENT_ENCODER_VALUE = 80;
    public static final int ADDRESS_BATTERY_VOLTAGE = 84;
    public static final int ADDRESS_MOTOR1_GEAR_RATIO = 86;
    public static final int ADDRESS_MOTOR1_P_COEFFICIENT = 87;
    public static final int ADDRESS_MOTOR1_I_COEFFICIENT = 88;
    public static final int ADDRESS_MOTOR1_D_COEFFICIENT = 89;
    public static final int ADDRESS_MOTOR2_GEAR_RATIO = 90;
    public static final int ADDRESS_MOTOR2_P_COEFFICIENT = 91;
    public static final int ADDRESS_MOTOR2_I_COEFFICIENT = 92;
    public static final int ADDRESS_MOTOR2_D_COEFFICIENT = 93;
    public static final int ADDRESS_UNUSED = 255;
    public static final int[] ADDRESS_MOTOR_POWER_MAP = new int[]{255, 69, 70};
    public static final int[] ADDRESS_MOTOR_MODE_MAP = new int[]{255, 68, 71};
    public static final int[] ADDRESS_MOTOR_TARGET_ENCODER_VALUE_MAP = new int[]{255, 64, 72};
    public static final int[] ADDRESS_MOTOR_CURRENT_ENCODER_VALUE_MAP = new int[]{255, 76, 80};
    public static final int[] ADDRESS_MOTOR_GEAR_RATIO_MAP = new int[]{255, 86, 90};
    public static final int[] ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP = new int[]{255, 87, 91};
    public static final int BUSY_THRESHOLD = 5;

    public ModernRoboticsUsbDcMotorController(SerialNumber serialNumber, RobotUsbDevice device, EventLoopManager manager) throws RobotCoreException, InterruptedException {
        super(serialNumber, manager, new ReadWriteRunnableBlocking(serialNumber, device, 30, 64, false));
        this.readWriteRunnable.setCallback(this);
        this.a();
        this.b();
    }

    @Override
    public String getDeviceName() {
        return "Modern Robotics USB DC Motor Controller";
    }

    public String getConnectionInfo() {
        return "USB " + (Object)this.getSerialNumber();
    }

    @Override
    public void close() {
        this.a();
        super.close();
    }

    public void setMotorControllerDeviceMode(DcMotorController.DeviceMode mode) {
    }

    public DcMotorController.DeviceMode getMotorControllerDeviceMode() {
        return DcMotorController.DeviceMode.READ_WRITE;
    }

    public void setMotorChannelMode(int motor, DcMotorController.RunMode mode) {
        this.a(motor);
        byte by = ModernRoboticsUsbDcMotorController.runModeToFlag(mode);
        this.write(ADDRESS_MOTOR_MODE_MAP[motor], by);
    }

    public DcMotorController.RunMode getMotorChannelMode(int motor) {
        this.a(motor);
        return ModernRoboticsUsbDcMotorController.flagToRunMode(this.read(ADDRESS_MOTOR_MODE_MAP[motor]));
    }

    public void setMotorPower(int motor, double power) {
        this.a(motor);
        Range.throwIfRangeIsInvalid((double)power, (double)-1.0, (double)1.0);
        this.write(ADDRESS_MOTOR_POWER_MAP[motor], new byte[]{(byte)(power * 100.0)});
    }

    public double getMotorPower(int motor) {
        this.a(motor);
        byte by = this.read(ADDRESS_MOTOR_POWER_MAP[motor]);
        double d = by == -128 ? 0.0 : (double)by / 100.0;
        return d;
    }

    public boolean isBusy(int motor) {
        this.a(motor);
        return Math.abs(this.getMotorTargetPosition(motor) - this.getMotorCurrentPosition(motor)) > 5;
    }

    public void setMotorPowerFloat(int motor) {
        this.a(motor);
        this.write(ADDRESS_MOTOR_POWER_MAP[motor], new byte[]{-128});
    }

    public boolean getMotorPowerFloat(int motor) {
        this.a(motor);
        byte by = this.read(ADDRESS_MOTOR_POWER_MAP[motor]);
        return by == -128;
    }

    public void setMotorTargetPosition(int motor, int position) {
        this.a(motor);
        Range.throwIfRangeIsInvalid((double)position, (double)-2.147483648E9, (double)2.147483647E9);
        this.write(ADDRESS_MOTOR_TARGET_ENCODER_VALUE_MAP[motor], TypeConversion.intToByteArray((int)position));
    }

    public int getMotorTargetPosition(int motor) {
        this.a(motor);
        byte[] arrby = this.read(ADDRESS_MOTOR_TARGET_ENCODER_VALUE_MAP[motor], 4);
        return TypeConversion.byteArrayToInt((byte[])arrby);
    }

    public int getMotorCurrentPosition(int motor) {
        this.a(motor);
        byte[] arrby = this.read(ADDRESS_MOTOR_CURRENT_ENCODER_VALUE_MAP[motor], 4);
        return TypeConversion.byteArrayToInt((byte[])arrby);
    }

    public double getVoltage() {
        byte[] arrby = this.read(84, 2);
        int n = TypeConversion.byteArrayToShort((byte[])arrby);
        n>>=6;
        double d = (double)(n&=1023) / 1023.0;
        return d * 20.4;
    }

    public void setGearRatio(int motor, double ratio) {
        this.a(motor);
        Range.throwIfRangeIsInvalid((double)ratio, (double)-1.0, (double)1.0);
        this.write(ADDRESS_MOTOR_GEAR_RATIO_MAP[motor], new byte[]{(byte)(ratio * 127.0)});
    }

    public double getGearRatio(int motor) {
        this.a(motor);
        byte[] arrby = this.read(ADDRESS_MOTOR_GEAR_RATIO_MAP[motor], 1);
        return (double)arrby[0] / 127.0;
    }

    public void setDifferentialControlLoopCoefficients(int motor, DifferentialControlLoopCoefficients pid) {
        this.a(motor);
        if (pid.p > 255.0) {
            pid.p = 255.0;
        }
        if (pid.i > 255.0) {
            pid.i = 255.0;
        }
        if (pid.d > 255.0) {
            pid.d = 255.0;
        }
        this.write(ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP[motor], new byte[]{(byte)pid.p, (byte)pid.i, (byte)pid.d});
    }

    public DifferentialControlLoopCoefficients getDifferentialControlLoopCoefficients(int motor) {
        this.a(motor);
        DifferentialControlLoopCoefficients differentialControlLoopCoefficients = new DifferentialControlLoopCoefficients();
        byte[] arrby = this.read(ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP[motor], 3);
        differentialControlLoopCoefficients.p = arrby[0];
        differentialControlLoopCoefficients.i = arrby[1];
        differentialControlLoopCoefficients.d = arrby[2];
        return differentialControlLoopCoefficients;
    }

    public static byte runModeToFlag(DcMotorController.RunMode mode) {
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

    public static DcMotorController.RunMode flagToRunMode(byte flag) {
        switch (flag & 3) {
            case 1: {
                return DcMotorController.RunMode.RUN_USING_ENCODERS;
            }
            case 0: {
                return DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
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

    private void a() {
        this.setMotorPowerFloat(1);
        this.setMotorPowerFloat(2);
    }

    private void b() {
        for (int i = 1; i <= 2; ++i) {
            this.write(ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP[i], new byte[]{-128, 64, -72});
        }
    }

    private void a(int n) {
        if (n < 1 || n > 2) {
            throw new IllegalArgumentException(String.format("Motor %d is invalid; valid motors are 1..%d", n, 2));
        }
    }

}

