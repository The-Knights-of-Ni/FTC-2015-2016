//datasheet for IMU: http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

//TODO: handle multiple devices on a single port

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cController;
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
    public static final byte PAGE_ID         = 0x07;
    public static final byte OPR_MODE        = 0x3D;
    public static final byte UNIT_SEL        = 0x3B;
    public static final byte SYS_TRIGGER     = 0x3F;
    public static final byte CHIP_ID         = 0x00;
    public static final byte PWR_MODE        = 0x3E;
    public static final byte SELFTEST_RESULT = 0x36;
    public static final byte CALIB_STAT      = 0x35;
    
    public static final byte ACC_CONFIG      = 0x08;
    
    public static final byte EUL_DATA_X      = 0x1A;
    public static final byte EUL_DATA_Y      = 0x1C;
    public static final byte EUL_DATA_Z      = 0x1E;
    
    public static final byte QUA_DATA_W      = 0x20;
    public static final byte QUA_DATA_X      = 0x22;
    public static final byte QUA_DATA_Y      = 0x24;
    public static final byte QUA_DATA_Z      = 0x26;
    
    public static final byte LIA_DATA_X      = 0x28;
    public static final byte LIA_DATA_Y      = 0x2A;
    public static final byte LIA_DATA_Z      = 0x2C;
    
    /* public static final byte GRV_DATA_X      = 0x2E; */
    /* public static final byte GRV_DATA_Y      = 0x30; */
    /* public static final byte GRV_DATA_Z      = 0x32; */
    
    public static final byte power_mode_normal  = 0b00;
    public static final byte power_mode_low     = 0b01;
    public static final byte power_mode_suspend = 0b10;    
    
    
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
    public static final byte units_acc_m_per_s2             =              0b0;
    public static final byte units_acc_g                    =              0b1;
    
    public static final byte units_angular_vel_deg_per_s    =             0b00;
    public static final byte units_angular_vel_rad_per_s    =             0b10;
    
    public static final byte units_angle_deg                =            0b000;
    public static final byte units_angle_rad                =            0b100;
    
    public static final byte units_temp_C                   =          0b00000;
    public static final byte units_temp_F                   =          0b10000;
    
    public static final byte units_pitch_convention_windows = (byte) 0b0000000;
    public static final byte units_pitch_convention_android = (byte) 0b1000000;
    
    //this many bytes at start of read_cache write_cache are system overhead,
    public static final int dib_cache_overhead = 4;
    public static final int action_flag = 31;
    
    public final static byte bCHIP_ID_VALUE = (byte) 0xA0;
    //////////////////////end constants//////////////////////
    
    public short eul_x = 0;
    public short eul_y = 0;
    public short eul_z = 0;
    
    public short qua_w = 0;
    public short qua_x = 0;
    public short qua_y = 0;
    public short qua_z = 0;
    
    public short lia_x = 0;
    public short lia_y = 0;
    public short lia_z = 0;
    
    //derived values
    public float acc_x = 0.0f;
    public float acc_y = 0.0f;
    public float acc_z = 0.0f;

    public float acc_x_low = 0.0f;
    public float acc_y_low = 0.0f;
    public float acc_z_low = 0.0f;
    
    public float acc_x_high = 0.0f;
    public float acc_y_high = 0.0f;
    public float acc_z_high = 0.0f;
    
    public float acc0_x = 0.0f;
    public float acc0_y = 0.0f;
    public float acc0_z = 0.0f;
    
    /* public float jerk_x = 0.0f; */
    /* public float jerk_y = 0.0f; */
    /* public float jerk_z = 0.0f; */
    
    public float vel_x = 0.0f;
    public float vel_y = 0.0f;
    public float vel_z = 0.0f;
    
    public int i2c_address = 0x28<<1; //the default address, modern robotics doubles the address for some reason
    public I2cDevice i2cd;
    
    public static final int widest_read_range = 27;
    
    public final byte[] read_cache;
    public final Lock read_lock;
    public final byte[] write_cache;
    public final Lock write_lock;
    
    public int highest_read_address = 0x2D;
    public int lowest_read_address = 0x1A;
    
    public int n_reads = 0;
    
    public long old_time;
    public double dt = 0;
    
    public IMU(I2cDevice I2CD)
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
    
    //TODO: add pre-set option to enable all reads from all readable registers
    public int init(byte mode, byte unit_flags)
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
            delay(10);
        }
        delay(50);
        DbgLog.error("imu reset");
        
        write8(PWR_MODE, power_mode_normal);
        delay(10);
        
        write8(PAGE_ID, (byte) 0);
        
        write8(UNIT_SEL, unit_flags);
        
        write8(SYS_TRIGGER, (byte)(slow_read8(SYS_TRIGGER) | (byte)0x01));
        for(int self_test_result = 0; (self_test_result & 0x0F) != 0x0F;)
        {
            self_test_result = slow_read8(SELFTEST_RESULT);
        }
        DbgLog.error("imu self tested");
        
        write8(OPR_MODE, mode);
        delay(200);

        write8(ACC_CONFIG, (byte) 0b00);
        //write8(ACC_CONFIG, (byte) 0b00011100);
        //write8(ACC_CONFIG, (byte) 0b00011100);
        
        write8(PAGE_ID, (byte) 0);
        
        while((slow_read8(CALIB_STAT)&0xF) != 0xF){delay(100);}
        
        /* lowest_read_address = (int) registers_to_read[0]; */
        /* highest_read_address = (int) registers_to_read[0]; */
        /* for(int r = 1; r < registers_to_read.length; r++) */
        /* { */
        /*     if(registers_to_read[r] < lowest_read_address) */
        /*     { */
        /*         lowest_read_address = (int) registers_to_read[r]; */
        /*     } */
        /*     if(registers_to_read[r] > highest_read_address) */
        /*     { */
        /*         highest_read_address = (int) registers_to_read[r]; */
        /*     } */
        /* } */
        if(highest_read_address+1-lowest_read_address > widest_read_range)
        {
            DbgLog.error(String.format("error: cannot read more than %d elements at a time", widest_read_range));
            return -1;
        }
        
        i2cd.enableI2cReadMode(i2c_address, lowest_read_address, highest_read_address+1-lowest_read_address);
        i2cd.setI2cPortActionFlag();
        i2cd.writeI2cPortFlagOnlyToController();
        for(int count = 0; count < 2; count++)
        {
            while(!i2cd.isI2cPortReady()){delay(5);}
            i2cd.setI2cPortActionFlag();
            i2cd.writeI2cPortFlagOnlyToController();
            i2cd.readI2cCacheFromController();
        }
        
        n_reads = 0;        
        
        old_time = System.nanoTime();
        
        return 0;
    }
    
    public void delay(int time)
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
    
    //read_lock must be obtained before calling this or any of the functions that use it
    public short shortFromCache(byte register)
    {
            return ByteBuffer.wrap(read_cache, register-lowest_read_address+dib_cache_overhead, 2).order(ByteOrder.nativeOrder()).getShort();
    }
    
    public byte slow_read8(int address)
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
    
    public void write(int address, byte[] data)
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
    
    public void write8(int address, byte value)
    {
        byte[] data = new byte[]{value};
        write(address, data);
    }
    
    float lerp(float a, float b, float t)
    {
        if(t > 1.0f) return b;
        if(t < 0.0f) return a;
        return a+(b-a)*t;
    }

    public void rezero()
    {
        vel_x = 0.0f;
        vel_y = 0.0f;
        vel_z = 0.0f;
        dt = 0.0f;
        old_time = System.nanoTime();
        n_reads = 0;//might not want to reset this, not sure
    }
    
    /* public static float filter_k = 0.5f; */
    /* public static float filter_c = 1.0f; */
    /* public static float filter_m = 1.0f; */
    
    /* public static float lambda = -filter_c/(2.0f*filter_m); */
    /* public static float omega = (float)Math.sqrt(4.0f*filter_k/filter_m-(filter_c*filter_c)/(filter_m*filter_m)); */
    
    /* public float[] filter(float x_1, float x_0, float v_0, float dt) */
    /* { */
    /*     float[] out = new float[2]; */

    /*     float c_1 = x_0-x_1; */
    /*     float c_2 = v_0/(omega*lambda); */
        
    /*     //NOTE: the small angle approx for sin and cos can probably be */
    /*     //used here if speed is needed, it probably won't make that large */
    /*     //of a difference since there's only 1 imu */
    /*     out[0] = x_1+(float)(Math.exp(lambda*dt)*(c_1*Math.cos(omega*dt) + c_2*Math.sin(omega*dt))); */
    /*     out[1] = (float)(omega*lambda*Math.exp(lambda*dt)*(-c_1*Math.sin(omega*dt) + c_2*Math.cos(omega*dt))); */
    /*     return out; */
    /* } */
    
    public static int n_reads_before_updating = 1<<6;
    
    float[] past_dts = new float[n_reads_before_updating];
    
    int[] past_acc_x = new int[n_reads_before_updating];
    int[] past_acc_y = new int[n_reads_before_updating];
    int[] past_acc_z = new int[n_reads_before_updating];
    
    int[] wavelet_acc_x = new int[n_reads_before_updating];
    int[] wavelet_acc_y = new int[n_reads_before_updating];
    int[] wavelet_acc_z = new int[n_reads_before_updating];

    float[] past_vel_x = new float[n_reads_before_updating];
    float[] past_vel_y = new float[n_reads_before_updating];
    float[] past_vel_z = new float[n_reads_before_updating];
    
    int n_past_acc = 0;
    
    int absi(int a)
    {
        if(a < 0) return -a;
        return a;
    }
    
    boolean filter(int acc, int[] past_acc, int[] wavelet_acc, float[] past_vel, float dt)
    {
        past_dts[n_past_acc] = dt;
        past_acc[n_past_acc] = acc;
        n_past_acc++;
        
        if(n_past_acc >= n_reads_before_updating)
        {
            haarWaveletTransform(past_acc, wavelet_acc, n_reads_before_updating);
            inverseHaarWaveletTransform(wavelet_acc, past_acc, n_reads_before_updating);
            for(int i = 1; i < n_reads_before_updating; i++)
            {
                past_vel[i] = past_vel[i-1]+past_acc[i]*past_dts[i];
            }
            past_vel[0] = past_vel[n_reads_before_updating-1];
            n_past_acc = 0;//TODO: make it more continuous
            return true;
        }
        return false;
    }
    
    //NOTE: the input array is not preserved, n must be a power of 2
    void haarWaveletTransform(int[] in, int[] out, int n)
    {
        for(int length = n>>1; ; length>>=1)
        {
            for(int i = 0; i < length; i++)
            {
                out[i] = (in[2*i]+in[2*i+1])>>1; //TODO: move the >>1's to the inverse transform unless it causes an int overflow
                out[i+length] = (in[2*i]-in[2*i+1])>>1;
                if(absi(out[i]) < 40) out[i] = 0; //TODO: try not filtering until the end
                if(absi(out[i+length]) < 40) out[i+length] = 0;
            }
            //return;//TEMP
            if(length == 1)
            {
                return;
            }
            System.arraycopy(out, 0, in, 0, length<<1);
        }
    }
    
    void inverseHaarWaveletTransform(int[] in, int[] out, int n)
    {
        for(int length = 1; ; length<<=1)
        {
            //length = n>>1;//TEMP
            for(int i = 0; i < length; i++)
            {
                out[2*i] = in[i]+in[i+length];
                out[2*i+1] = in[i]-in[i+length];
            }
            //return;//TEMP
            if(length == n>>1)
            {
                return;
            }
            System.arraycopy(out, 0, in, 0, length<<1);
        }
    }
    
    public boolean checkForUpdate()
    {
        if(i2cd.isI2cPortReady())
        {
            i2cd.setI2cPortActionFlag();
            i2cd.writeI2cPortFlagOnlyToController();
            i2cd.readI2cCacheFromController();
            
            try
            {
                read_lock.lock();

                //timer
                long new_time = System.nanoTime();
                dt = 0.000000001f*(float)(new_time-old_time);
                old_time = new_time;
                
                eul_x = shortFromCache(EUL_DATA_X);
                eul_y = shortFromCache(EUL_DATA_Y);
                eul_z = shortFromCache(EUL_DATA_Z);
                
                qua_w = shortFromCache(QUA_DATA_W);
                qua_x = shortFromCache(QUA_DATA_X);
                qua_y = shortFromCache(QUA_DATA_Y);
                qua_z = shortFromCache(QUA_DATA_Z);
                
                lia_x = shortFromCache(LIA_DATA_X);
                lia_y = shortFromCache(LIA_DATA_Y);
                lia_z = shortFromCache(LIA_DATA_Z);
                
                if(filter(lia_x, past_acc_x, wavelet_acc_x, past_vel_x, (float) dt))
                {
                    acc_x = past_acc_x[n_reads_before_updating-1];
                    vel_x = past_vel_x[n_reads_before_updating-1];
                }
                if(filter(lia_y, past_acc_y, wavelet_acc_y, past_vel_y, (float) dt))
                {
                    acc_y = past_acc_y[n_reads_before_updating-1];
                    vel_y = past_vel_y[n_reads_before_updating-1];
                }
                if(filter(lia_z, past_acc_z, wavelet_acc_z, past_vel_z, (float) dt))
                {
                    acc_z = past_acc_z[n_reads_before_updating-1];
                    vel_z = past_vel_z[n_reads_before_updating-1];
                }
                /* float[] filtered_x = filter(((float) lia_x)-acc0_x, acc_x, jerk_x, (float) dt); */
                /* acc_x = filtered_x[0]; */
                /* jerk_x = filtered_x[1]; */
                
                /* float[] filtered_y = filter(((float) lia_y)-acc0_y, acc_y, jerk_y, (float) dt); */
                /* acc_y = filtered_y[0]; */
                /* jerk_y = filtered_y[1]; */
                
                /* float[] filtered_z = filter(((float) lia_z)-acc0_z, acc_z, jerk_z, (float) dt); */
                /* acc_z = filtered_z[0]; */
                /* jerk_z = filtered_z[1]; */
                
                /* vel_x += acc_x*dt; */
                /* vel_y += acc_y*dt; */
                /* vel_z += acc_z*dt; */
                
                n_reads++; //so you can check if there is a new value since you last checked,
                           //might not be necessary with the manual update function
            }
            finally
            {
                read_lock.unlock();
            }
            return true;
        }
        return false;
    }
}
