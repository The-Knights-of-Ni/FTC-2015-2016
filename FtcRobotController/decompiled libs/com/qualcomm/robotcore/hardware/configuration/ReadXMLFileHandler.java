/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  org.xmlpull.v1.XmlPullParser
 *  org.xmlpull.v1.XmlPullParserException
 *  org.xmlpull.v1.XmlPullParserFactory
 */
package com.qualcomm.robotcore.hardware.configuration;

import android.content.Context;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceInterfaceModuleConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MatrixControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;
import org.xmlpull.v1.XmlPullParserFactory;

public class ReadXMLFileHandler {
    private static boolean b = false;
    List<ControllerConfiguration> a = new ArrayList<ControllerConfiguration>();
    private static int c = 2;
    private static int d = 8;
    private static int e = 8;
    private static int f = 2;
    private static int g = 6;
    private static int h = 6;
    private static int i = 6;
    private static int j = 2;
    private static int k = 4;
    private static int l = 4;
    private XmlPullParser m;

    public ReadXMLFileHandler(Context context) {
    }

    public List<ControllerConfiguration> getDeviceControllers() {
        return this.a;
    }

    public List<ControllerConfiguration> parse(InputStream is) throws RobotCoreException {
        this.m = null;
        try {
            XmlPullParserFactory xmlPullParserFactory = XmlPullParserFactory.newInstance();
            xmlPullParserFactory.setNamespaceAware(true);
            this.m = xmlPullParserFactory.newPullParser();
            this.m.setInput(is, null);
            int n = this.m.getEventType();
            while (n != 1) {
                String string = this.a(this.m.getName());
                if (n == 2) {
                    if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER.toString())) {
                        this.a.add(this.b(true));
                    }
                    if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER.toString())) {
                        this.a.add(this.a(true));
                    }
                    if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.LEGACY_MODULE_CONTROLLER.toString())) {
                        this.a.add(this.b());
                    }
                    if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.DEVICE_INTERFACE_MODULE.toString())) {
                        this.a.add(this.a());
                    }
                }
                n = this.m.next();
            }
        }
        catch (XmlPullParserException var3_4) {
            RobotLog.w("XmlPullParserException");
            var3_4.printStackTrace();
        }
        catch (IOException var3_5) {
            RobotLog.w("IOException");
            var3_5.printStackTrace();
        }
        return this.a;
    }

    private ControllerConfiguration a() throws IOException, XmlPullParserException, RobotCoreException {
        String string = this.m.getAttributeValue(null, "name");
        String string2 = this.m.getAttributeValue(null, "serialNumber");
        ArrayList<DeviceConfiguration> arrayList = this.a(c, DeviceConfiguration.ConfigurationType.PULSE_WIDTH_DEVICE);
        ArrayList<DeviceConfiguration> arrayList2 = this.a(g, DeviceConfiguration.ConfigurationType.I2C_DEVICE);
        ArrayList<DeviceConfiguration> arrayList3 = this.a(e, DeviceConfiguration.ConfigurationType.ANALOG_INPUT);
        ArrayList<DeviceConfiguration> arrayList4 = this.a(d, DeviceConfiguration.ConfigurationType.DIGITAL_DEVICE);
        ArrayList<DeviceConfiguration> arrayList5 = this.a(f, DeviceConfiguration.ConfigurationType.ANALOG_OUTPUT);
        int n = this.m.next();
        String string3 = this.a(this.m.getName());
        while (n != 1) {
            DeviceConfiguration deviceConfiguration;
            if (n == 3) {
                if (string3 == null) continue;
                if (b) {
                    RobotLog.e("[handleDeviceInterfaceModule] tagname: " + string3);
                }
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.DEVICE_INTERFACE_MODULE.toString())) {
                    deviceConfiguration = new DeviceInterfaceModuleConfiguration(string, new SerialNumber(string2));
                    deviceConfiguration.setPwmDevices(arrayList);
                    deviceConfiguration.setI2cDevices(arrayList2);
                    deviceConfiguration.setAnalogInputDevices(arrayList3);
                    deviceConfiguration.setDigitalDevices(arrayList4);
                    deviceConfiguration.setAnalogOutputDevices(arrayList5);
                    deviceConfiguration.setEnabled(true);
                    return deviceConfiguration;
                }
            }
            if (n == 2) {
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.ANALOG_INPUT.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.OPTICAL_DISTANCE_SENSOR.toString())) {
                    deviceConfiguration = this.c();
                    arrayList3.set(deviceConfiguration.getPort(), deviceConfiguration);
                }
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.PULSE_WIDTH_DEVICE.toString())) {
                    deviceConfiguration = this.c();
                    arrayList.set(deviceConfiguration.getPort(), deviceConfiguration);
                }
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.I2C_DEVICE.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.IR_SEEKER_V3.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.ADAFRUIT_COLOR_SENSOR.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.COLOR_SENSOR.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.GYRO.toString())) {
                    deviceConfiguration = this.c();
                    arrayList2.set(deviceConfiguration.getPort(), deviceConfiguration);
                }
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.ANALOG_OUTPUT.toString())) {
                    deviceConfiguration = this.c();
                    arrayList5.set(deviceConfiguration.getPort(), deviceConfiguration);
                }
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.DIGITAL_DEVICE.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.TOUCH_SENSOR.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.LED.toString())) {
                    deviceConfiguration = this.c();
                    arrayList4.set(deviceConfiguration.getPort(), deviceConfiguration);
                }
            }
            n = this.m.next();
            string3 = this.a(this.m.getName());
        }
        RobotLog.logAndThrow("Reached the end of the XML file while parsing.");
        return null;
    }

    private ControllerConfiguration b() throws IOException, XmlPullParserException, RobotCoreException {
        String string = this.m.getAttributeValue(null, "name");
        String string2 = this.m.getAttributeValue(null, "serialNumber");
        ArrayList<DeviceConfiguration> arrayList = this.a(h, DeviceConfiguration.ConfigurationType.NOTHING);
        int n = this.m.next();
        String string3 = this.a(this.m.getName());
        while (n != 1) {
            DeviceConfiguration deviceConfiguration;
            if (n == 3) {
                if (string3 == null) continue;
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.LEGACY_MODULE_CONTROLLER.toString())) {
                    deviceConfiguration = new LegacyModuleControllerConfiguration(string, arrayList, new SerialNumber(string2));
                    deviceConfiguration.setEnabled(true);
                    return deviceConfiguration;
                }
            }
            if (n == 2) {
                if (b) {
                    RobotLog.e("[handleLegacyModule] tagname: " + string3);
                }
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.COMPASS.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.LIGHT_SENSOR.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.IR_SEEKER.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.ACCELEROMETER.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.GYRO.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.TOUCH_SENSOR.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.TOUCH_SENSOR_MULTIPLEXER.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.ULTRASONIC_SENSOR.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.COLOR_SENSOR.toString()) || string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.NOTHING.toString())) {
                    deviceConfiguration = this.c();
                    arrayList.set(deviceConfiguration.getPort(), deviceConfiguration);
                } else if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER.toString())) {
                    deviceConfiguration = this.b(false);
                    arrayList.set(deviceConfiguration.getPort(), deviceConfiguration);
                } else if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER.toString())) {
                    deviceConfiguration = this.a(false);
                    arrayList.set(deviceConfiguration.getPort(), deviceConfiguration);
                } else if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MATRIX_CONTROLLER.toString())) {
                    deviceConfiguration = this.d();
                    arrayList.set(deviceConfiguration.getPort(), deviceConfiguration);
                }
            }
            n = this.m.next();
            string3 = this.a(this.m.getName());
        }
        return new LegacyModuleControllerConfiguration(string, arrayList, new SerialNumber(string2));
    }

    private DeviceConfiguration c() {
        String string = this.a(this.m.getName());
        int n = Integer.parseInt(this.m.getAttributeValue(null, "port"));
        DeviceConfiguration deviceConfiguration = new DeviceConfiguration(n);
        deviceConfiguration.setType(deviceConfiguration.typeFromString(string));
        deviceConfiguration.setName(this.m.getAttributeValue(null, "name"));
        if (!deviceConfiguration.getName().equalsIgnoreCase("NO DEVICE ATTACHED")) {
            deviceConfiguration.setEnabled(true);
        }
        if (b) {
            RobotLog.e("[handleDevice] name: " + deviceConfiguration.getName() + ", port: " + deviceConfiguration.getPort() + ", type: " + (Object)deviceConfiguration.getType());
        }
        return deviceConfiguration;
    }

    private ArrayList<DeviceConfiguration> a(int n, DeviceConfiguration.ConfigurationType configurationType) {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < n; ++i) {
            if (configurationType == DeviceConfiguration.ConfigurationType.SERVO) {
                arrayList.add(new ServoConfiguration(i + 1, "NO DEVICE ATTACHED", false));
                continue;
            }
            if (configurationType == DeviceConfiguration.ConfigurationType.MOTOR) {
                arrayList.add(new MotorConfiguration(i + 1, "NO DEVICE ATTACHED", false));
                continue;
            }
            arrayList.add(new DeviceConfiguration(i, configurationType, "NO DEVICE ATTACHED", false));
        }
        return arrayList;
    }

    private ControllerConfiguration d() throws IOException, XmlPullParserException, RobotCoreException {
        String string = this.m.getAttributeValue(null, "name");
        String string2 = ControllerConfiguration.NO_SERIAL_NUMBER.toString();
        int n = Integer.parseInt(this.m.getAttributeValue(null, "port"));
        ArrayList<DeviceConfiguration> arrayList = this.a(l, DeviceConfiguration.ConfigurationType.SERVO);
        ArrayList<DeviceConfiguration> arrayList2 = this.a(k, DeviceConfiguration.ConfigurationType.MOTOR);
        int n2 = this.m.next();
        String string3 = this.a(this.m.getName());
        while (n2 != 1) {
            if (n2 == 3) {
                if (string3 == null) continue;
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MATRIX_CONTROLLER.toString())) {
                    MatrixControllerConfiguration matrixControllerConfiguration = new MatrixControllerConfiguration(string, arrayList2, arrayList, new SerialNumber(string2));
                    matrixControllerConfiguration.setPort(n);
                    matrixControllerConfiguration.setEnabled(true);
                    return matrixControllerConfiguration;
                }
            }
            if (n2 == 2) {
                int n3;
                String string4;
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO.toString())) {
                    n3 = Integer.parseInt(this.m.getAttributeValue(null, "port"));
                    string4 = this.m.getAttributeValue(null, "name");
                    ServoConfiguration servoConfiguration = new ServoConfiguration(n3, string4, true);
                    arrayList.set(n3 - 1, servoConfiguration);
                } else if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR.toString())) {
                    n3 = Integer.parseInt(this.m.getAttributeValue(null, "port"));
                    string4 = this.m.getAttributeValue(null, "name");
                    MotorConfiguration motorConfiguration = new MotorConfiguration(n3, string4, true);
                    arrayList2.set(n3 - 1, motorConfiguration);
                }
            }
            n2 = this.m.next();
            string3 = this.a(this.m.getName());
        }
        RobotLog.logAndThrow("Reached the end of the XML file while parsing.");
        return null;
    }

    private ControllerConfiguration a(boolean bl) throws IOException, XmlPullParserException {
        String string = this.m.getAttributeValue(null, "name");
        int n = -1;
        String string2 = ControllerConfiguration.NO_SERIAL_NUMBER.toString();
        if (bl) {
            string2 = this.m.getAttributeValue(null, "serialNumber");
        } else {
            n = Integer.parseInt(this.m.getAttributeValue(null, "port"));
        }
        ArrayList<DeviceConfiguration> arrayList = this.a(i, DeviceConfiguration.ConfigurationType.SERVO);
        int n2 = this.m.next();
        String string3 = this.a(this.m.getName());
        while (n2 != 1) {
            if (n2 == 3) {
                if (string3 == null) continue;
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER.toString())) {
                    ServoControllerConfiguration servoControllerConfiguration = new ServoControllerConfiguration(string, arrayList, new SerialNumber(string2));
                    servoControllerConfiguration.setPort(n);
                    servoControllerConfiguration.setEnabled(true);
                    return servoControllerConfiguration;
                }
            }
            if (n2 == 2 && string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO.toString())) {
                int n3 = Integer.parseInt(this.m.getAttributeValue(null, "port"));
                String string4 = this.m.getAttributeValue(null, "name");
                ServoConfiguration servoConfiguration = new ServoConfiguration(n3, string4, true);
                arrayList.set(n3 - 1, servoConfiguration);
            }
            n2 = this.m.next();
            string3 = this.a(this.m.getName());
        }
        ServoControllerConfiguration servoControllerConfiguration = new ServoControllerConfiguration(string, arrayList, new SerialNumber(string2));
        servoControllerConfiguration.setPort(n);
        return servoControllerConfiguration;
    }

    private ControllerConfiguration b(boolean bl) throws IOException, XmlPullParserException {
        String string = this.m.getAttributeValue(null, "name");
        int n = -1;
        String string2 = ControllerConfiguration.NO_SERIAL_NUMBER.toString();
        if (bl) {
            string2 = this.m.getAttributeValue(null, "serialNumber");
        } else {
            n = Integer.parseInt(this.m.getAttributeValue(null, "port"));
        }
        ArrayList<DeviceConfiguration> arrayList = this.a(j, DeviceConfiguration.ConfigurationType.MOTOR);
        int n2 = this.m.next();
        String string3 = this.a(this.m.getName());
        while (n2 != 1) {
            if (n2 == 3) {
                if (string3 == null) continue;
                if (string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER.toString())) {
                    MotorControllerConfiguration motorControllerConfiguration = new MotorControllerConfiguration(string, arrayList, new SerialNumber(string2));
                    motorControllerConfiguration.setPort(n);
                    motorControllerConfiguration.setEnabled(true);
                    return motorControllerConfiguration;
                }
            }
            if (n2 == 2 && string3.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR.toString())) {
                int n3 = Integer.parseInt(this.m.getAttributeValue(null, "port"));
                String string4 = this.m.getAttributeValue(null, "name");
                MotorConfiguration motorConfiguration = new MotorConfiguration(n3, string4, true);
                arrayList.set(n3 - 1, motorConfiguration);
            }
            n2 = this.m.next();
            string3 = this.a(this.m.getName());
        }
        MotorControllerConfiguration motorControllerConfiguration = new MotorControllerConfiguration(string, arrayList, new SerialNumber(string2));
        motorControllerConfiguration.setPort(n);
        return motorControllerConfiguration;
    }

    private String a(String string) {
        if (string == null) {
            return null;
        }
        if (string.equalsIgnoreCase("MotorController")) {
            return DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER.toString();
        }
        if (string.equalsIgnoreCase("ServoController")) {
            return DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER.toString();
        }
        if (string.equalsIgnoreCase("LegacyModuleController")) {
            return DeviceConfiguration.ConfigurationType.LEGACY_MODULE_CONTROLLER.toString();
        }
        if (string.equalsIgnoreCase("DeviceInterfaceModule")) {
            return DeviceConfiguration.ConfigurationType.DEVICE_INTERFACE_MODULE.toString();
        }
        if (string.equalsIgnoreCase("AnalogInput")) {
            return DeviceConfiguration.ConfigurationType.ANALOG_INPUT.toString();
        }
        if (string.equalsIgnoreCase("OpticalDistanceSensor")) {
            return DeviceConfiguration.ConfigurationType.OPTICAL_DISTANCE_SENSOR.toString();
        }
        if (string.equalsIgnoreCase("IrSeeker")) {
            return DeviceConfiguration.ConfigurationType.IR_SEEKER.toString();
        }
        if (string.equalsIgnoreCase("LightSensor")) {
            return DeviceConfiguration.ConfigurationType.LIGHT_SENSOR.toString();
        }
        if (string.equalsIgnoreCase("DigitalDevice")) {
            return DeviceConfiguration.ConfigurationType.DIGITAL_DEVICE.toString();
        }
        if (string.equalsIgnoreCase("TouchSensor")) {
            return DeviceConfiguration.ConfigurationType.TOUCH_SENSOR.toString();
        }
        if (string.equalsIgnoreCase("IrSeekerV3")) {
            return DeviceConfiguration.ConfigurationType.IR_SEEKER_V3.toString();
        }
        if (string.equalsIgnoreCase("PulseWidthDevice")) {
            return DeviceConfiguration.ConfigurationType.PULSE_WIDTH_DEVICE.toString();
        }
        if (string.equalsIgnoreCase("I2cDevice")) {
            return DeviceConfiguration.ConfigurationType.I2C_DEVICE.toString();
        }
        if (string.equalsIgnoreCase("AnalogOutput")) {
            return DeviceConfiguration.ConfigurationType.ANALOG_OUTPUT.toString();
        }
        if (string.equalsIgnoreCase("TouchSensorMultiplexer")) {
            return DeviceConfiguration.ConfigurationType.TOUCH_SENSOR_MULTIPLEXER.toString();
        }
        if (string.equalsIgnoreCase("MatrixController")) {
            return DeviceConfiguration.ConfigurationType.MATRIX_CONTROLLER.toString();
        }
        if (string.equalsIgnoreCase("UltrasonicSensor")) {
            return DeviceConfiguration.ConfigurationType.ULTRASONIC_SENSOR.toString();
        }
        if (string.equalsIgnoreCase("AdafruitColorSensor")) {
            return DeviceConfiguration.ConfigurationType.ADAFRUIT_COLOR_SENSOR.toString();
        }
        if (string.equalsIgnoreCase("ColorSensor")) {
            return DeviceConfiguration.ConfigurationType.COLOR_SENSOR.toString();
        }
        if (string.equalsIgnoreCase("Led")) {
            return DeviceConfiguration.ConfigurationType.LED.toString();
        }
        if (string.equalsIgnoreCase("Gyro")) {
            return DeviceConfiguration.ConfigurationType.GYRO.toString();
        }
        return string;
    }
}

