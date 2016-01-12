/*
WARNING: this is a generated file
changes made this file are not permanent
*/
package com.qualcomm.ftcrobotcontroller.opmodes;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class Mk3TeleopRobotStateElements
{
    public static byte[] robot_state;
    public static int robot_state_size = 84;
    
    Mk3TeleopRobotStateElements(){}
    public static void set_left_drive(float value)
    {
        ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_left_drive()
    {
        return ByteBuffer.wrap(robot_state, 0, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_right_drive(float value)
    {
        ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_right_drive()
    {
        return ByteBuffer.wrap(robot_state, 4, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_elbow(float value)
    {
        ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_elbow()
    {
        return ByteBuffer.wrap(robot_state, 8, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_shoulder(float value)
    {
        ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_shoulder()
    {
        return ByteBuffer.wrap(robot_state, 12, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_intake(float value)
    {
        ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_intake()
    {
        return ByteBuffer.wrap(robot_state, 16, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_hand(float value)
    {
        ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_hand()
    {
        return ByteBuffer.wrap(robot_state, 20, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_slide(float value)
    {
        ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_slide()
    {
        return ByteBuffer.wrap(robot_state, 24, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick1_x(float value)
    {
        ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick1_x()
    {
        return ByteBuffer.wrap(robot_state, 28, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick1_y(float value)
    {
        ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick1_y()
    {
        return ByteBuffer.wrap(robot_state, 32, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick2_x(float value)
    {
        ByteBuffer.wrap(robot_state, 36, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick2_x()
    {
        return ByteBuffer.wrap(robot_state, 36, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_joystick2_y(float value)
    {
        ByteBuffer.wrap(robot_state, 40, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_joystick2_y()
    {
        return ByteBuffer.wrap(robot_state, 40, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_trigger1(float value)
    {
        ByteBuffer.wrap(robot_state, 44, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_trigger1()
    {
        return ByteBuffer.wrap(robot_state, 44, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_trigger2(float value)
    {
        ByteBuffer.wrap(robot_state, 48, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad1_trigger2()
    {
        return ByteBuffer.wrap(robot_state, 48, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad1_buttons(int value)
    {
        ByteBuffer.wrap(robot_state, 52, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_gamepad1_buttons()
    {
        return ByteBuffer.wrap(robot_state, 52, 4).order(ByteOrder.nativeOrder()).getInt();
    }

    public static void set_gamepad2_joystick1_x(float value)
    {
        ByteBuffer.wrap(robot_state, 56, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick1_x()
    {
        return ByteBuffer.wrap(robot_state, 56, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_joystick1_y(float value)
    {
        ByteBuffer.wrap(robot_state, 60, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick1_y()
    {
        return ByteBuffer.wrap(robot_state, 60, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_joystick2_x(float value)
    {
        ByteBuffer.wrap(robot_state, 64, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick2_x()
    {
        return ByteBuffer.wrap(robot_state, 64, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_joystick2_y(float value)
    {
        ByteBuffer.wrap(robot_state, 68, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_joystick2_y()
    {
        return ByteBuffer.wrap(robot_state, 68, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_trigger1(float value)
    {
        ByteBuffer.wrap(robot_state, 72, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_trigger1()
    {
        return ByteBuffer.wrap(robot_state, 72, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_trigger2(float value)
    {
        ByteBuffer.wrap(robot_state, 76, 4).order(ByteOrder.nativeOrder()).putFloat(value);
    }

    public static float get_gamepad2_trigger2()
    {
        return ByteBuffer.wrap(robot_state, 76, 4).order(ByteOrder.nativeOrder()).getFloat();
    }

    public static void set_gamepad2_buttons(int value)
    {
        ByteBuffer.wrap(robot_state, 80, 4).order(ByteOrder.nativeOrder()).putInt(value);
    }

    public static int get_gamepad2_buttons()
    {
        return ByteBuffer.wrap(robot_state, 80, 4).order(ByteOrder.nativeOrder()).getInt();
    }

}