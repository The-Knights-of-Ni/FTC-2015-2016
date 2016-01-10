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
    public static int robot_state_size = 100;
    
    camera_testRobotStateElements(){}
    public static byte[] get_camera_buffer()
    {
        return ByteBuffer.wrap(robot_state, 0, 100).order(ByteOrder.nativeOrder()).array();
    }

}