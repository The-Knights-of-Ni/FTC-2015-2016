/*
WARNING: this is a generated file
changes made this file are not permanent
*/
package com.qualcomm.ftcrobotcontroller.opmodes;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class Mk3AutoRobotStateElements
{
    public static byte[] robot_state;
    public static int robot_state_size = 40;
    
    Mk3AutoRobotStateElements(){}
    public static void set_left_drive_encoder(int value)
    {
        ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_left_drive_encoder()
    {
        return ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_shoulder_encoder(int value)
    {
        ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_shoulder_encoder()
    {
        return ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_heading(float value)
    {
        ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_heading()
    {
        return ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_roll(float value)
    {
        ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_roll()
    {
        return ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_y_velocity(float value)
    {
        ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_y_velocity()
    {
        return ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_left_drive(float value)
    {
        ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_left_drive()
    {
        return ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_elbow(float value)
    {
        ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_elbow()
    {
        return ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_intake(float value)
    {
        ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_intake()
    {
        return ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_slide(float value)
    {
        ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_slide()
    {
        return ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_indicator(int value)
    {
        ByteBuffer.wrap(robot_state, 36, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_indicator()
    {
        return ByteBuffer.wrap(robot_state, 36, 4).order(ByteOrder.nativeOrder()).getInt();
    }

}