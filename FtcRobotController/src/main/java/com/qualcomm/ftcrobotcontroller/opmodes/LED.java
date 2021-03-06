package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.concurrent.locks.Lock;

/*
There are a bunch of ways to handle these, but not all of them work
i2c - The start and ack bits cause problems that make start frames impossible
digital - Likely too slow
analog out - Clock at 5000 Hz, Data is voltage on other line, might not work
Nothing's working right now.
 */
public class LED {
    DeviceInterfaceModule dim;
    final int dataPin;
    final int clockPin;

    public LED(DeviceInterfaceModule dim, int clockPin, int dataPin) {
        this.dim = dim;
        this.dataPin = dataPin;
        this.clockPin = clockPin;
        initClock();
    }

    public void delay(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            //TODO:
        }
    }
    
    public void initClock()//TODO: Use a square wave at 5V and 5000Hz, figure out if timing is possible
    {
        dim.setAnalogOutputMode(clockPin, (byte) 2);//Set port 0 to square wave
        dim.setAnalogOutputFrequency(clockPin, 5000);//Set port 0 to 5000 hz, max value
        dim.setAnalogOutputVoltage(clockPin, 5);//Set port 0 to 5v output
        //TODO: Figure out which of these should go last for timing, since I don't think I can handle it in any other way
    }

    public void write8(byte value)
    {
        initClock();
        delay(3);//Adjust this offset to control which bytes are written
        dim.setAnalogOutputMode(dataPin, (byte) 2);//Set Port to square wave
        dim.setAnalogOutputFrequency(dataPin, 5000);//Set port to 5000 hz
        dim.setAnalogOutputVoltage(dataPin, 5);//Set to 5v
        delay(1);//Wait 8 cycles and then kill it
        dim.setAnalogOutputVoltage(dataPin, 0);
    }

    public void startFrame() {
        write8((byte) 0);
        write8((byte) 0);
        write8((byte) 0);
        write8((byte) 0);
    }

    public void ledFrame(int r, int g, int b) {
        write8((byte) 255);
        write8((byte) 255);
        write8((byte) 255);
        write8((byte) 0);
    }
}
