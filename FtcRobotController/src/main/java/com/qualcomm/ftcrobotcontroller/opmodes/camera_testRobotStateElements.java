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
    public static int robot_state_size = 153600;
    
    camera_testRobotStateElements(){}
    public static byte[] get_camera_buffer()
    {
        return ByteBuffer.wrap(robot_state, 0, 76800).order(ByteOrder.nativeOrder()).array();
    }

    public static byte[] get_overlay_buffer()
    {
        return ByteBuffer.wrap(robot_state, 76800, 76800).order(ByteOrder.nativeOrder()).array();
    }

}