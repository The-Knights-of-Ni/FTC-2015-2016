/*
WARNING: this is a generated file
changes made this file are not permanent
*/
package com.qualcomm.ftcrobotcontroller.opmodes;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class testRobotStateElements
{
    public static byte[] robot_state;
    public static int robot_state_size = 136;
    
    testRobotStateElements(){}
    public static void set_left_drive_power(float value)
    {
        ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_left_drive_power()
    {
        return ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_right_drive_power(float value)
    {
        ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_right_drive_power()
    {
        return ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick1_x(float value)
    {
        ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick1_x()
    {
        return ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick1_y(float value)
    {
        ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick1_y()
    {
        return ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick2_x(float value)
    {
        ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick2_x()
    {
        return ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick2_y(float value)
    {
        ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick2_y()
    {
        return ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_trigger1(float value)
    {
        ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_trigger1()
    {
        return ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_trigger2(float value)
    {
        ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_trigger2()
    {
        return ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_buttons(int value)
    {
        ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_gamepad1_buttons()
    {
        return ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static byte[] get_array_example()
    {
        return ByteBuffer.wrap(robot_state, 36, 100).order(ByteOrder.nativeOrder()).array();
    }

}