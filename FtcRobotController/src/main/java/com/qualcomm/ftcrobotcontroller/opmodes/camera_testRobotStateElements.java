/*
WARNING: this is a generated file
changes made this file are not permanent
*/
package com.qualcomm.ftcrobotcontroller.opmodes;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class camera_testRobotStateElements
{
    public static byte[] robot_state;
    public static int robot_state_size = 9830404;
    
    camera_testRobotStateElements(){}
    public static byte[] get_camera_buffers()
    {
        return ByteBuffer.wrap(robot_state, 0, 9830400).order(ByteOrder.nativeOrder()).array();
    }

    public static void set_current_buffer(int value)
    {
        ByteBuffer.wrap(robot_state, 9830400, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_current_buffer()
    {
        return ByteBuffer.wrap(robot_state, 9830400, 4).order(ByteOrder.nativeOrder()).getInt();
    }

}