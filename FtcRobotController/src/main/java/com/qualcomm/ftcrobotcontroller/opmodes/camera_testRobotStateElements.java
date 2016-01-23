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
<<<<<<< HEAD
    public static int robot_state_size = 100;
    
    camera_testRobotStateElements(){}
    public static byte[] get_camera_buffer()
    {
        return ByteBuffer.wrap(robot_state, 0, 100).order(ByteOrder.nativeOrder()).array();
=======
    public static int robot_state_size = 8;
    
    camera_testRobotStateElements(){}
    public static void set_camera_w(int value)
    {
        ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_camera_w()
    {
        return ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_camera_h(int value)
    {
        ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_camera_h()
    {
        return ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).getInt();
>>>>>>> camera
    }

}