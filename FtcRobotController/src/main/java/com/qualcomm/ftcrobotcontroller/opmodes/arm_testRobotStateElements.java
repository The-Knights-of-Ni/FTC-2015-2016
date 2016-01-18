/*
WARNING: this is a generated file
changes made this file are not permanent
*/
package com.qualcomm.ftcrobotcontroller.opmodes;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class arm_testRobotStateElements
{
    public static byte[] robot_state;
    public static int robot_state_size = 88;
    
    arm_testRobotStateElements(){}
    public static void set_time(double value)
    {
        ByteBuffer.wrap(robot_state, 0, 8).order(ByteOrder.nativeOrder()).putDouble(value);
    }

    public static double get_time()
    {
        return ByteBuffer.wrap(robot_state, 0, 8).order(ByteOrder.nativeOrder()).getDouble();
    }

    public static void set_arm_shoulder_power(float value)
    {
        ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_arm_shoulder_power()
    {
        return ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_arm_winch_power(float value)
    {
        ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_arm_winch_power()
    {
        return ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_arm_intake_power(float value)
    {
        ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_arm_intake_power()
    {
        return ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick1_x(float value)
    {
        ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick1_x()
    {
        return ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick1_y(float value)
    {
        ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick1_y()
    {
        return ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick2_x(float value)
    {
        ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick2_x()
    {
        return ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick2_y(float value)
    {
        ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick2_y()
    {
        return ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_left_trigger(float value)
    {
        ByteBuffer.wrap(robot_state, 36, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_left_trigger()
    {
        return ByteBuffer.wrap(robot_state, 36, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_right_trigger(float value)
    {
        ByteBuffer.wrap(robot_state, 40, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_right_trigger()
    {
        return ByteBuffer.wrap(robot_state, 40, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_buttons(int value)
    {
        ByteBuffer.wrap(robot_state, 44, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_gamepad1_buttons()
    {
        return ByteBuffer.wrap(robot_state, 44, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_gamepad2_joystick1_x(float value)
    {
        ByteBuffer.wrap(robot_state, 48, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick1_x()
    {
        return ByteBuffer.wrap(robot_state, 48, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_joystick1_y(float value)
    {
        ByteBuffer.wrap(robot_state, 52, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick1_y()
    {
        return ByteBuffer.wrap(robot_state, 52, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_joystick2_x(float value)
    {
        ByteBuffer.wrap(robot_state, 56, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick2_x()
    {
        return ByteBuffer.wrap(robot_state, 56, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_joystick2_y(float value)
    {
        ByteBuffer.wrap(robot_state, 60, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick2_y()
    {
        return ByteBuffer.wrap(robot_state, 60, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_left_trigger(float value)
    {
        ByteBuffer.wrap(robot_state, 64, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_left_trigger()
    {
        return ByteBuffer.wrap(robot_state, 64, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_right_trigger(float value)
    {
        ByteBuffer.wrap(robot_state, 68, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_right_trigger()
    {
        return ByteBuffer.wrap(robot_state, 68, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_buttons(int value)
    {
        ByteBuffer.wrap(robot_state, 72, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_gamepad2_buttons()
    {
        return ByteBuffer.wrap(robot_state, 72, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_shoulder_encoder(float value)
    {
        ByteBuffer.wrap(robot_state, 76, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_shoulder_encoder()
    {
        return ByteBuffer.wrap(robot_state, 76, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_elbow_potentiometer(float value)
    {
        ByteBuffer.wrap(robot_state, 80, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_elbow_potentiometer()
    {
        return ByteBuffer.wrap(robot_state, 80, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_winch_encoder(float value)
    {
        ByteBuffer.wrap(robot_state, 84, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_winch_encoder()
    {
        return ByteBuffer.wrap(robot_state, 84, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

}