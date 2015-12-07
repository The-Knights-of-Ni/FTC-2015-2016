//datasheet for IMU: http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

//TODO: handle multiple devices on a single port

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.concurrent.locks.Lock;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class IMU
    implements I2cController.I2cPortReadyCallback
{
    public int port;
    public int address = 0x28; //the default address
    public DeviceInterfaceModule dim;
    
    IMU(int PORT, DeviceInterfaceModule DIM)
    {
        PORT = port;
        dim = DIM;
        this.b = dim.getI2cReadCache(port);
        this.c = dim.getI2cReadCacheLock(port);

        dim.setI2cPortActionFlag(port);

    }

    public static final byte OPR_MODE = 0x3B;
    public static final byte UNIT_SEL = 0x3D;
    
    
    public static final byte EUL_DATA_X = 0x1A;
    
    
    public static final byte mode_config       = 0b0000;

    public static final byte mode_acconly      = 0b0001;
    public static final byte mode_magonly      = 0b0010;
    public static final byte mode_gyroonly     = 0b0011;
    public static final byte mode_accmag       = 0b0100;
    public static final byte mode_accgyro      = 0b0101;
    public static final byte mode_maggyro      = 0b0110;
    public static final byte mode_amg          = 0b0111;
    
    public static final byte mode_imu          = 0b1000;
    public static final byte mode_compass      = 0b1001;
    public static final byte mode_m4g          = 0b1010;
    public static final byte mode_ndof_fmc_off = 0b1011;
    public static final byte mode_ndof         = 0b1100;
    

    //bitwise or the units_ together to get unit_flags
    public static final byte units_acc_m_per_s2             =        0b0;
    public static final byte units_acc_g                    =        0b1;
    
    public static final byte units_angular_vel_deg_per_s    =       0b00;
    public static final byte units_angular_vel_rad_per_s    =       0b10;
    
    public static final byte units_angle_deg                =      0b000;
    public static final byte units_angle_rad                =      0b100;
    
    public static final byte units_temp_C                   =    0b00000;
    public static final byte units_temp_F                   =    0b10000;
    
    public static final byte units_pitch_convention_windows = (byte) 0b00000000;
    public static final byte units_pitch_convention_android = (byte) 0b10000000;

    private final byte[] b;
    private final Lock c;

    
    void init(byte mode, byte unit_flags)
    {
        byte[] mode_buffer = new byte[]{mode};
        dim.enableI2cWriteMode(port, address, OPR_MODE, mode_buffer.length);
        dim.copyBufferIntoWriteBuffer(port, mode_buffer);
        
        byte[] unit_flags_buffer = new byte[]{unit_flags};
        dim.enableI2cWriteMode(port, address, UNIT_SEL, unit_flags_buffer.length);
        dim.copyBufferIntoWriteBuffer(port, unit_flags_buffer);
        
        dim.writeI2cCacheToController(port);
        dim.registerForI2cPortReadyCallback((I2cController.I2cPortReadyCallback) this, port);
    }

    short getEulerHeading()
    {
        short out;
        try {
            c.lock();
            out = ByteBuffer.wrap(b, 0, 2).order(ByteOrder.nativeOrder()).getShort();
        }
        finally
        {
            c.unlock();
        }
        return out;
    }

    private int a(int n) {
        byte by;
        try {
            this.c.lock();
            by = this.b[n];
        }
        finally {
            this.c.unlock();
        }
        return TypeConversion.unsignedByteToInt((byte)by);
    }

    public void portIsReady(int PORT)
    {
        dim.setI2cPortActionFlag(port);
        dim.readI2cCacheFromController(port);
        {
            dim.enableI2cReadMode(port, address, EUL_DATA_X, 2);
            dim.writeI2cCacheToController(port);
        }
    }
}
