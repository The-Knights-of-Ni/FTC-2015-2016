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
    public static int robot_state_size = 152;
    
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

    public static void set_gamepad1_left_trigger(float value)
    {
        ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_left_trigger()
    {
        return ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_right_trigger(float value)
    {
        ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_right_trigger()
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

    public static void set_left_bumper(int value)
    {
        ByteBuffer.wrap(robot_state, 136, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_left_bumper()
    {
        return ByteBuffer.wrap(robot_state, 136, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_right_bumper(int value)
    {
        ByteBuffer.wrap(robot_state, 140, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_right_bumper()
    {
        return ByteBuffer.wrap(robot_state, 140, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_toggle_left_bumper(int value)
    {
        ByteBuffer.wrap(robot_state, 144, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_toggle_left_bumper()
    {
        return ByteBuffer.wrap(robot_state, 144, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_toggle_right_bumper(int value)
    {
        ByteBuffer.wrap(robot_state, 148, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_toggle_right_bumper()
    {
        return ByteBuffer.wrap(robot_state, 148, 4).order(ByteOrder.nativeOrder()).getInt();
    }

}