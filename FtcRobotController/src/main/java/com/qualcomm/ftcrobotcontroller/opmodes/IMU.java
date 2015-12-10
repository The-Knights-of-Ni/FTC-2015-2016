//datasheet for IMU: http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

//TODO: handle multiple devices on a single port

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.I2cDevice;
import java.util.concurrent.locks.Lock;
import com.qualcomm.ftccommon.DbgLog;

//import java.util.concurrent.atomic.AtomicIntegerArray;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/*
  device interface notes:
  when port is ready you can either read or write data, or just write a port flag only if there is nothing to do
*/

public class IMU
{
    //////////////////////constants//////////////////////////
    //register addresses
    public static final byte PAGE_ID = 0x07;
    public static final byte OPR_MODE = 0x3D;
    public static final byte UNIT_SEL = 0x3B;
    public static final byte SYS_TRIGGER = 0x3F;
    public static final byte CHIP_ID = 0x00;
    public static final byte PWR_MODE = 0x3E;
    public static final byte SELFTEST_RESULT = 0x36;

    public static final byte power_mode_normal  = 0b00;
    public static final byte power_mode_low     = 0b01;
    public static final byte power_mode_suspend = 0b10;
    
    
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
    public static final byte units_acc_m_per_s2             =               0b0;
    public static final byte units_acc_g                    =               0b1;
    
    public static final byte units_angular_vel_deg_per_s    =              0b00;
    public static final byte units_angular_vel_rad_per_s    =              0b10;
    
    public static final byte units_angle_deg                =             0b000;
    public static final byte units_angle_rad                =             0b100;
    
    public static final byte units_temp_C                   =           0b00000;
    public static final byte units_temp_F                   =           0b10000;
    
    public static final byte units_pitch_convention_windows = (byte) 0b00000000;
    public static final byte units_pitch_convention_android = (byte) 0b10000000;

    //this many bytes at start of read_cache write_cache are system overhead,
    public static final int dib_cache_overhead = 4;
    public static final int action_flag = 31;

    public final static byte bCHIP_ID_VALUE = (byte) 0xA0;
//////////////////////end constants//////////////////////

    
    public int i2c_address = 0x28<<1; //the default address, modern robotics doubles the address for some reason
    public I2cDevice i2cd;
    
    //TODO: allow for the read ranges to be non-adjacent
    //TODO: handle reading from registers on page 2
    public static final int widest_read_range = 26;
    
    private final byte[] read_cache;
    private final Lock read_lock;
    private final byte[] write_cache;
    private final Lock write_lock;
    
    int highest_read_address;
    int lowest_read_address;
    
    boolean ready_for_update = false;
    int in_read_mode = 0;
    
    IMU(I2cDevice I2CD)
    {
        i2cd = I2CD;
        
        this.read_cache  = i2cd.getI2cReadCache();
        this.read_lock   = i2cd.getI2cReadCacheLock();
        this.write_cache = i2cd.getI2cWriteCache();
        this.write_lock  = i2cd.getI2cWriteCacheLock();
        
        i2cd.setI2cPortActionFlag();
        i2cd.writeI2cCacheToController();
    }
        
    /*for manual checking with core device discovery:

      write8(0x07, (byte)0);
      read8(0x00); //make sure it's 160
      write8(0x3D, 0x00);
      write8(0x3F, (byte) 0x20);
      read8(0x00); //make sure it's 160
      write8(0x3E, 0x00);
      write8(0x07, (byte) 0);
      write8(0x3F, (byte)(read8(0x3F) | (byte)0x01));
      read8(0x36); //make sure it's < 0x0F
      write8(0x3D, 0b1100);
      
      
      write8(0x07, (byte)0);
      read8(0x00); //make sure it's 160
      write8(0x3D, mode_config);
      write8(0x3F, (byte)0x20);
      read8(0x00); //make sure it's 160
      write8(0x3E, power_mode_normal);
      write8(0x07, (byte) 0);
      write(0x3B, unit_flags_buffer);
      write8(0x3F, (byte)(read8(SYS_TRIGGER) | (byte)0x01));
      read8(0x36); //make sure it's < 0x0F
      write8(0x3D, mode);

    */
    
    //TODO: add preset option to enable all reads from all readable registers
    int init(byte mode, byte unit_flags, byte[] registers_to_read)
    {
        DbgLog.error("imu init");
        
        write8(PAGE_ID, (byte)0);
        DbgLog.error("wrote 0 to PAGE_ID");
        
        byte chip_id = slow_read8(CHIP_ID);
        DbgLog.error(String.format("chip_id 0x%X", chip_id));
        if (chip_id != bCHIP_ID_VALUE)
        {
            delay(650); //delay value is from from Table 0-2
            chip_id = slow_read8(CHIP_ID);
            DbgLog.error(String.format("chip_id 0x%X", chip_id));
            
            if (chip_id != bCHIP_ID_VALUE)
            {
                DbgLog.error("wrong chip id");
                return -1;
            }
        }
        
        write8(OPR_MODE, mode_config);
        delay(30);
        
        //reset
        write8(SYS_TRIGGER, (byte)0x20);    
        DbgLog.error("waiting for reset");
        for(int i = 0;; i++)
        {
            if(i >= 10)
            {
                DbgLog.error("error during IMU reset");
                return -1;
            }
            chip_id = slow_read8(CHIP_ID);
            if(chip_id == bCHIP_ID_VALUE) break;
            delay(100);
        }
        delay(50);
        DbgLog.error("imu reset");

        write8(PAGE_ID, (byte) 0);
        
        write8(PWR_MODE, power_mode_normal);
        delay(10);
        
        write8(UNIT_SEL, unit_flags);
        
        write8(SYS_TRIGGER, (byte)(slow_read8(SYS_TRIGGER) | (byte)0x01));
        for(int self_test_result = 0; (self_test_result & 0x0F) != 0x0F;)
        {
            self_test_result = slow_read8(SELFTEST_RESULT);
        }        
        DbgLog.error("imu self tested");
        
        write8(OPR_MODE, mode);
        delay(200);

        lowest_read_address = (int) registers_to_read[0];
        highest_read_address = (int) registers_to_read[0];
        for(int r = 1; r < registers_to_read.length; r++)
        {
            if(registers_to_read[r] < lowest_read_address)
            {
                lowest_read_address = (int) registers_to_read[r];
            }
            if(registers_to_read[r] > highest_read_address)
            {
                highest_read_address = (int) registers_to_read[r];
            }
        }        
        if(highest_read_address-lowest_read_address > widest_read_range)
        {
            DbgLog.error(String.format("error: cannot read more than %d elements at a time", widest_read_range));
            return -1;
        }
        
        return 0;
    }
    
    void delay(int time)
    {
        /* long start_time = System.nanoTime(); */
        /* while((System.nanoTime()-start_time)/1000 < time +50){} */
        try
        {
            Thread.sleep(time);
        }
        catch (InterruptedException e)
        {
            //TODO:
        }
    }

    boolean checkForUpdate()
    {
        if(!ready_for_update)
        {
            i2cd.enableI2cReadMode(i2c_address, lowest_read_address, highest_read_address-lowest_read_address);
            i2cd.setI2cPortActionFlag();
            i2cd.writeI2cPortFlagOnlyToController();
            i2cd.readI2cCacheFromController();
            ready_for_update = true;
        }
        else if(i2cd.isI2cPortReady())
        {
            if(in_read_mode > 0)
            {
                i2cd.setI2cPortActionFlag();
                i2cd.writeI2cPortFlagOnlyToController();
                i2cd.readI2cCacheFromController();
                return true;
            }
            else if(i2cd.isI2cPortInReadMode())
            {
                i2cd.setI2cPortActionFlag();
                i2cd.writeI2cPortFlagOnlyToController();
                i2cd.readI2cCacheFromController();
                in_read_mode++;
            }
            else
            {
                i2cd.readI2cCacheFromController();
            }
        }
        return false;
    }
    
    short getEulerHeading()
    {
        try
        {
            read_lock.lock();
            DbgLog.error("Heading low part: "+String.format("%d", read_cache[dib_cache_overhead]));
            return ByteBuffer.wrap(read_cache, EUL_DATA_X-lowest_read_address+dib_cache_overhead, 2).order(ByteOrder.nativeOrder()).getShort();
        }
        finally
        {
            read_lock.unlock();
        }
    }
    
    byte slow_read8(int address)
    {
        i2cd.enableI2cReadMode(i2c_address, address, 1);
        while(!i2cd.isI2cPortReady())
        {
            delay(100);
        }
        i2cd.setI2cPortActionFlag();
        i2cd.writeI2cPortFlagOnlyToController();
        i2cd.readI2cCacheFromController();
        while(!i2cd.isI2cPortInReadMode())
        {
            delay(100);
            i2cd.readI2cCacheFromController();
        }
        try
        {
            read_lock.lock();
            DbgLog.error(String.format("Read 0x%X: 0x%X", address, read_cache[dib_cache_overhead]));
            return read_cache[dib_cache_overhead];
        }
        finally
        {
            read_lock.unlock();
        }
    }
    
    void write(int address, byte[] data)
    {
        while(!i2cd.isI2cPortReady())
        {
            delay(100);
        }
        
        //copy data to write cache
        try
        {
            write_lock.lock();
            System.arraycopy(data, 0,
                             write_cache, dib_cache_overhead,
                             data.length);
        }
        finally
        {
            write_lock.unlock();
        }
        
        DbgLog.error("writing to " + String.format("0x%02X", address));
        DbgLog.error("value " + String.format("0x%02X, %d", data[0], data.length));
        
        i2cd.setI2cPortActionFlag();
        i2cd.enableI2cWriteMode(i2c_address,
                                address,
                                data.length);
        i2cd.writeI2cCacheToController();
    }
    
    void write8(int address, byte value)
    {
        byte[] data = new byte[]{value};
        write(address, data);
    }
}
