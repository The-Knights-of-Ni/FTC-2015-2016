//datasheet for IMU: http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

//TODO: handle multiple devices on a single port

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.concurrent.locks.Lock;
import com.qualcomm.ftccommon.DbgLog;

//import java.util.concurrent.atomic.AtomicIntegerArray;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/*
  device interface notes:
  when port is ready you can either read or write data, or just write a port flag only if there is nothing to do
*/

public class IMU implements I2cController.I2cPortReadyCallback
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
    //////////////////////end constants//////////////////////
    
    
    public int port;
    public int address = 0x28<<1; //the default address, modern robotics doubles the address for some reason
    public I2cDevice i2cd;
    
    //TODO: allow for the read ranges to be non-adjacent
    //TODO: handle reading from registers on page 2
    volatile int lowest_read_address;
    volatile int highest_read_address;
    int current_read_address = 0;
    public static final int widest_read_range = 26;
    volatile int n_reads;
    volatile int reads_per_cycle;
    volatile byte[] sensor_read_cache; //volatile is only to ensure we have the correct reference if the read settings are changed
    boolean read_waiting = false;
    boolean read_ready = false;
    
    //this many bytes at start of read_cache write_cache are system overhead,
    public static final int dib_cache_overhead = 4;
    private static final int action_flag = 31;
    
    private final byte[] read_cache;
    private final Lock read_lock;
    private final byte[] write_cache;
    private final Lock write_lock;
    
    int write_queue_front = 0;
    int write_queue_back = 0;
    public static final int write_queue_max_length = 100; //TODO: might need to adjust this size
    int[] write_queue_start_registers = new int[write_queue_max_length];
    byte[][] write_queue_data = new byte[write_queue_max_length][];
    
    IMU(int PORT, I2cDevice I2CD)
    {
        PORT = port;
        i2cd = I2CD;
        
        this.read_cache = i2cd.getI2cReadCache();
        this.read_lock = i2cd.getI2cReadCacheLock();
        this.write_cache = i2cd.getI2cWriteCache();
        this.write_lock = i2cd.getI2cWriteCacheLock();
        
        i2cd.setI2cPortActionFlag();
        i2cd.writeI2cCacheToController();
    }
    
    public final static byte bCHIP_ID_VALUE = (byte) 0xA0;
    
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

        /* byte[] registers_to_read_for_init = new byte[]{ */
        /*     PAGE_ID, */
        /*     //SYS_TRIGGER, */
        /*     CHIP_ID, */
        /*     //SELFTEST_RESULT, */
        /* }; */
        
        /* //find read range */
        /* lowest_read_address = (int) registers_to_read_for_init[0]; */
        /* highest_read_address = (int) registers_to_read_for_init[0]; */
        
        /* for(int r = 1; r < registers_to_read_for_init.length; r++) */
        /* { */
        /*     if(registers_to_read_for_init[r] < lowest_read_address) */
        /*     { */
        /*         lowest_read_address = (int) registers_to_read_for_init[r]; */
        /*     } */
        /*     if(registers_to_read_for_init[r] > highest_read_address) */
        /*     { */
        /*         highest_read_address = (int) registers_to_read_for_init[r]; */
        /*     } */
        /* } */
        /* n_reads = 0; */
        /* reads_per_cycle = (highest_read_address+1-lowest_read_address+widest_read_range-1)/widest_read_range; */
        /* sensor_read_cache = new byte[highest_read_address+1-lowest_read_address]; */
        /* i2cd.enableI2cReadMode(address, lowest_read_address, highest_read_address-lowest_read_address);//TEMP */
        /* while(!i2cd.isI2cPortInReadMode()){delay(100);} */
        
        /* DbgLog.error("lowest address = " + String.format("%d", lowest_read_address)); */
        /* DbgLog.error("highest address = " + String.format("%d", highest_read_address)); */
        
        /* //i2cd.registerForI2cPortReadyCallback((I2cController.I2cPortReadyCallback) this); */
        
        /* DbgLog.error("imu init"); */
        /* write8(PAGE_ID, (byte)0); */
        /* DbgLog.error("wrote 0 to PAGE_ID"); */
        
        /* //wait for all values to read once */
        /* while(n_reads < 1*reads_per_cycle) */
        /* { */
        /*     delay(100); */
        /*     //DbgLog.error("waiting for reads"); */
        /* } */
        
        /* synchronized(sensor_read_cache) */
        /* { */
        /*     for(int i = 0; i < sensor_read_cache.length; i++) */
        /*     { */
        /*         DbgLog.error("sensor cache "+String.format("0x%02x = 0x%02x", i, sensor_read_cache[i])); */
        /*     } */
        /* } */
        
        /* byte chip_id = read8(CHIP_ID); */
        /* DbgLog.error(String.format("chip_id 0x%x", chip_id)); */
        /* if (chip_id != bCHIP_ID_VALUE) */
        /* { */
        /*     DbgLog.error("beginning wait"); */
        /*     int old_n_reads = n_reads; */
        /*     delay(650); //delay value is from from Table 0-2 */
        /*     while(n_reads <= old_n_reads+reads_per_cycle){delay(10);} */
        /*     DbgLog.error("done waiting"); */
        /*     chip_id = read8(CHIP_ID); */
        /*     DbgLog.error(String.format("chip_id 0x%x", chip_id)); */
        /*     if (chip_id != bCHIP_ID_VALUE) */
        /*     { */
        /*         DbgLog.error("wrong chip id"); */
        /*         return -1; */
        /*     } */
        /* } */
        
        write8(OPR_MODE, mode_config);
        delay(30);
        
        //reset
        write8(SYS_TRIGGER, (byte)0x20);    
        /* for(int old_n_reads = n_reads; n_reads <= old_n_reads+reads_per_cycle; delay(100)){} *///TODO:
        DbgLog.error("waiting for reset");
        /* for(int i = 0;; i++) */
        /* { */
        /*     //if(i >= 10) return -1; */
        /*     chip_id = read8(CHIP_ID); */
        /*     if(chip_id == bCHIP_ID_VALUE) break; */
        /*     delay(10); */
        /* } */
        /* delay(50); */
        DbgLog.error("imu reset");
        
        write8(PWR_MODE, power_mode_normal);
        delay(10);

        write8(PAGE_ID, (byte) 0);
        
        write8(UNIT_SEL, unit_flags);
        
        /* write8(SYS_TRIGGER, (byte)(read8(SYS_TRIGGER) | (byte)0x01)); */
        /* for(int self_test_result = 0; (self_test_result & 0x0F) != 0x0F;) */
        /* { */
        /*     self_test_result = read8(SELFTEST_RESULT); */
        /* } */
        
        /* DbgLog.error("imu self tested"); */
        
        write8(OPR_MODE, mode);
        delay(200);
        
        //find read range
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
        n_reads = 0;
        reads_per_cycle = (highest_read_address+1-lowest_read_address+widest_read_range-1)/widest_read_range;
        sensor_read_cache = new byte[highest_read_address+1-lowest_read_address];
        i2cd.enableI2cReadMode(address, lowest_read_address, highest_read_address-lowest_read_address);//TEMP
        while(!i2cd.isI2cPortReady()){delay(1000);}
        
        i2cd.registerForI2cPortReadyCallback((I2cController.I2cPortReadyCallback) this);
        /* while(n_reads < reads_per_cycle) */
        /* { */
        /*     delay(10); */
        /* } */
        
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
    
    short getEulerHeading()
    {
        return ByteBuffer.wrap(sensor_read_cache, EUL_DATA_X-lowest_read_address, 2).order(ByteOrder.nativeOrder()).getShort();
    }
    
    byte read8(int address)
    {
        synchronized(sensor_read_cache)
        {
            return sensor_read_cache[address-lowest_read_address];
        }
    }
    
    void write(int address, byte[] data)
    {
        synchronized(write_queue_data)
        {
            assert(write_queue_front+1 != write_queue_back);
            write_queue_start_registers[write_queue_back] = address;
            write_queue_data[write_queue_back] = data;
            write_queue_back = (write_queue_back+1)%write_queue_max_length;
            DbgLog.error("write command issued");

            //TEMP
            while(!i2cd.isI2cPortReady())
            {
                delay(100);
            }
            
            i2cd.setI2cPortActionFlag();
            read_ready = false;
            read_waiting = false;
            DbgLog.error("writing to " + String.format("0x%02x", write_queue_start_registers[write_queue_front]));
            DbgLog.error("value " + String.format("0x%02x, %d", write_queue_data[write_queue_front][0], write_queue_data[write_queue_front].length));
            i2cd.enableI2cWriteMode(address,
                                    write_queue_start_registers[write_queue_front],
                                    write_queue_data[write_queue_front].length);
                
            //copy data to write cache
            try
            {
                write_lock.lock();
                System.arraycopy(write_queue_data[write_queue_front], 0, write_cache, dib_cache_overhead, write_queue_data[write_queue_front].length);
            }
            finally
            {
                write_lock.unlock();
            }
            write_queue_front = (write_queue_front+1) % write_queue_max_length; //remove the front element from the queue
                
            i2cd.writeI2cCacheToController();
        }
    }
    
    void write8(int address, byte value)
    {
        byte[] data = new byte[]{value};
        write(address, data);
    }
    
    @Override public void portIsReady(int PORT)
    {                    
        i2cd.setI2cPortActionFlag();
        i2cd.writeI2cPortFlagOnlyToController();
        i2cd.readI2cCacheFromController();

        try
        {
            read_lock.lock();
            synchronized(sensor_read_cache)
            {
                for(int i = 0; i < highest_read_address-lowest_read_address; i++)
                {
                    //DbgLog.error("read cache "+String.format("0x%02x = 0x%02x", i+current_read_address-lowest_read_address, read_cache[i+dib_cache_overhead]));
                }
                        
                System.arraycopy(read_cache, dib_cache_overhead,
                                 sensor_read_cache, 0,
                                 highest_read_address-lowest_read_address);
            }
        }
        finally
        {
            read_lock.unlock();
        }
        n_reads++;
        
        //DbgLog.error("port is ready");
        /* synchronized(write_queue_data) */
        /* { */
        /*     if(false) //write_queue_back != write_queue_front) */
        /*     { */
        /*         i2cd.setI2cPortActionFlag(); */
        /*         read_ready = false; */
        /*         read_waiting = false; */
        /*         DbgLog.error("writing to " + String.format("0x%02x", write_queue_start_registers[write_queue_front])); */
        /*         DbgLog.error("value " + String.format("0x%02x, %d", write_queue_data[write_queue_front][0], write_queue_data[write_queue_front].length)); */
        /*         i2cd.enableI2cWriteMode(address, */
        /*                                 write_queue_start_registers[write_queue_front], */
        /*                                 write_queue_data[write_queue_front].length); */
                
        /*         //copy data to write cache */
        /*         try */
        /*         { */
        /*             write_lock.lock(); */
        /*             System.arraycopy(write_queue_data[write_queue_front], 0, write_cache, dib_cache_overhead, write_queue_data[write_queue_front].length); */
        /*         } */
        /*         finally */
        /*         { */
        /*             write_lock.unlock(); */
        /*         } */
        /*         write_queue_front = (write_queue_front+1) % write_queue_max_length; //remove the front element from the queue */
                
        /*         i2cd.writeI2cCacheToController(); */
        /*         i2cd.readI2cCacheFromController(); */
        /*     } */
        /*     else */
        /*     { */
        /*         //DbgLog.error("reading"); */
        /*         int n_bytes = highest_read_address+1-current_read_address; //+1 because we want to include highest_read_address */
        /*         if(n_bytes > widest_read_range) n_bytes = widest_read_range; */
                
        /*         if(!read_waiting) */
        /*         { */
        /*             //DbgLog.error("starting read"); */
        /*             read_waiting = true; */
        /*             i2cd.enableI2cReadMode(address, current_read_address, n_bytes); */
                    
        /*             i2cd.setI2cPortActionFlag(); */
        /*             i2cd.writeI2cPortFlagOnlyToController(); */
        /*             i2cd.readI2cCacheFromController(); */
        /*         } */
        /*         else if(!read_ready && i2cd.isI2cPortInReadMode() && read_waiting) */
        /*         { */
        /*             //DbgLog.error("read waiting"); */
        /*             i2cd.setI2cPortActionFlag(); */
        /*             i2cd.writeI2cPortFlagOnlyToController(); */
        /*             i2cd.readI2cCacheFromController(); */
        /*             read_ready = true; */
        /*         } */
        /*         else if(read_ready) */
        /*         { */
        /*             read_ready = false; */
        /*             read_waiting = false; */
        /*             i2cd.setI2cPortActionFlag(); */
        /*             i2cd.writeI2cPortFlagOnlyToController(); */
        /*             i2cd.readI2cCacheFromController(); */
                    
        /*             //DbgLog.error("read ready"); */
                    
        /*             try */
        /*             { */
        /*                 read_lock.lock(); */
        /*                 synchronized(sensor_read_cache) */
        /*                 { */
        /*                     for(int i = 0; i < n_bytes; i++) */
        /*                     { */
        /*                         DbgLog.error("read cache "+String.format("0x%02x = 0x%02x", i+current_read_address-lowest_read_address, read_cache[i+dib_cache_overhead])); */
        /*                     } */
                        
        /*                     System.arraycopy(read_cache, dib_cache_overhead, */
        /*                                      sensor_read_cache, current_read_address-lowest_read_address, */
        /*                                      n_bytes); */
        /*                 } */
        /*             } */
        /*             finally */
        /*             { */
        /*                 read_lock.unlock(); */
        /*             } */
                
        /*             n_reads++; //increase the number of reads */
        /*             if(current_read_address+n_bytes > highest_read_address) */
        /*             { */
        /*                 //DbgLog.error("full cycle complete"); */
        /*                 current_read_address = lowest_read_address; */
        /*             } */
        /*             else */
        /*             { */
        /*                 current_read_address = current_read_address+n_bytes; */
        /*             } */
        /*         } */
        /*         else */
        /*         { */
        /*             i2cd.readI2cCacheFromController(); */

        /*             try */
        /*             { */
        /*                 write_lock.lock(); */
        /*                 write_cache[action_flag] = 0; */
        /*             } */
        /*             finally */
        /*             { */
        /*                 write_lock.unlock(); */
        /*             } */
        /*         } */
        /*     } */
        /* } */
    }
}
