/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Activity
 *  android.content.Context
 *  android.os.Bundle
 *  android.view.View
 *  android.view.View$OnClickListener
 *  android.widget.Button
 *  android.widget.LinearLayout
 *  android.widget.Toast
 *  com.qualcomm.ftccommon.R
 *  com.qualcomm.ftccommon.R$id
 *  com.qualcomm.ftccommon.R$layout
 *  com.qualcomm.ftccommon.R$string
 *  com.qualcomm.hardware.HardwareDeviceManager
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.DeviceManager
 *  com.qualcomm.robotcore.hardware.DeviceManager$DeviceType
 *  com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration$ConfigurationType
 *  com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MotorConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.ServoConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.Utility
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.ftccommon.configuration;

import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.Toast;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.R;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.SerialNumber;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class AutoConfigureActivity
extends Activity {
    private Context a;
    private Button b;
    private Button c;
    private DeviceManager d;
    protected Map<SerialNumber, DeviceManager.DeviceType> scannedDevices = new HashMap<SerialNumber, DeviceManager.DeviceType>();
    protected Set<Map.Entry<SerialNumber, DeviceManager.DeviceType>> entries = new HashSet<Map.Entry<SerialNumber, DeviceManager.DeviceType>>();
    private Map<SerialNumber, ControllerConfiguration> e = new HashMap<SerialNumber, ControllerConfiguration>();
    private Thread f;
    private Utility g;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.a = this;
        this.setContentView(R.layout.activity_autoconfigure);
        this.g = new Utility((Activity)this);
        this.b = (Button)this.findViewById(R.id.configureLegacy);
        this.c = (Button)this.findViewById(R.id.configureUSB);
        try {
            this.d = new HardwareDeviceManager(this.a, null);
        }
        catch (RobotCoreException var2_2) {
            this.g.complainToast("Failed to open the Device Manager", this.a);
            DbgLog.error("Failed to open deviceManager: " + var2_2.toString());
            DbgLog.logStacktrace((Exception)var2_2);
        }
    }

    protected void onStart() {
        super.onStart();
        this.g.updateHeader("K9LegacyBot", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
        String string = this.g.getFilenameFromPrefs(R.string.pref_hardware_config_filename, "No current file!");
        if (string.equals("K9LegacyBot") || string.equals("K9USBBot")) {
            this.d();
        } else {
            this.a();
        }
        this.b.setOnClickListener(new View.OnClickListener(){

            public void onClick(View view) {
                AutoConfigureActivity.this.f = new Thread(new Runnable(){

                    @Override
                    public void run() {
                        try {
                            DbgLog.msg("Scanning USB bus");
                            AutoConfigureActivity.this.scannedDevices = AutoConfigureActivity.this.d.scanForUsbDevices();
                        }
                        catch (RobotCoreException var1_1) {
                            DbgLog.error("Device scan failed");
                        }
                        AutoConfigureActivity.this.runOnUiThread(new Runnable(){

                            @Override
                            public void run() {
                                AutoConfigureActivity.this.g.resetCount();
                                if (AutoConfigureActivity.this.scannedDevices.size() == 0) {
                                    AutoConfigureActivity.this.g.saveToPreferences("No current file!", R.string.pref_hardware_config_filename);
                                    AutoConfigureActivity.this.g.updateHeader("No current file!", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                                    AutoConfigureActivity.this.a();
                                }
                                AutoConfigureActivity.this.entries = AutoConfigureActivity.this.scannedDevices.entrySet();
                                AutoConfigureActivity.this.e = new HashMap();
                                AutoConfigureActivity.this.g.createLists(AutoConfigureActivity.this.entries, AutoConfigureActivity.this.e);
                                boolean bl = AutoConfigureActivity.this.g();
                                if (bl) {
                                    AutoConfigureActivity.this.a("K9LegacyBot");
                                } else {
                                    AutoConfigureActivity.this.g.saveToPreferences("No current file!", R.string.pref_hardware_config_filename);
                                    AutoConfigureActivity.this.g.updateHeader("No current file!", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                                    AutoConfigureActivity.this.b();
                                }
                            }
                        });
                    }

                });
                AutoConfigureActivity.this.f.start();
            }

        });
        this.c.setOnClickListener(new View.OnClickListener(){

            public void onClick(View view) {
                AutoConfigureActivity.this.f = new Thread(new Runnable(){

                    @Override
                    public void run() {
                        try {
                            DbgLog.msg("Scanning USB bus");
                            AutoConfigureActivity.this.scannedDevices = AutoConfigureActivity.this.d.scanForUsbDevices();
                        }
                        catch (RobotCoreException var1_1) {
                            DbgLog.error("Device scan failed");
                        }
                        AutoConfigureActivity.this.runOnUiThread(new Runnable(){

                            @Override
                            public void run() {
                                AutoConfigureActivity.this.g.resetCount();
                                if (AutoConfigureActivity.this.scannedDevices.size() == 0) {
                                    AutoConfigureActivity.this.g.saveToPreferences("No current file!", R.string.pref_hardware_config_filename);
                                    AutoConfigureActivity.this.g.updateHeader("No current file!", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                                    AutoConfigureActivity.this.a();
                                }
                                AutoConfigureActivity.this.entries = AutoConfigureActivity.this.scannedDevices.entrySet();
                                AutoConfigureActivity.this.e = new HashMap();
                                AutoConfigureActivity.this.g.createLists(AutoConfigureActivity.this.entries, AutoConfigureActivity.this.e);
                                boolean bl = AutoConfigureActivity.this.e();
                                if (bl) {
                                    AutoConfigureActivity.this.a("K9USBBot");
                                } else {
                                    AutoConfigureActivity.this.g.saveToPreferences("No current file!", R.string.pref_hardware_config_filename);
                                    AutoConfigureActivity.this.g.updateHeader("No current file!", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                                    AutoConfigureActivity.this.c();
                                }
                            }
                        });
                    }

                });
                AutoConfigureActivity.this.f.start();
            }

        });
    }

    private void a(String string) {
        this.g.writeXML(this.e);
        try {
            this.g.writeToFile(string + ".xml");
            this.g.saveToPreferences(string, R.string.pref_hardware_config_filename);
            this.g.updateHeader(string, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
            Toast.makeText((Context)this.a, (CharSequence)("AutoConfigure " + string + " Successful"), (int)0).show();
        }
        catch (RobotCoreException var2_2) {
            this.g.complainToast(var2_2.getMessage(), this.a);
            DbgLog.error(var2_2.getMessage());
        }
        catch (IOException var2_3) {
            this.g.complainToast("Found " + var2_3.getMessage() + "\n Please fix and re-save", this.a);
            DbgLog.error(var2_3.getMessage());
        }
    }

    private void a() {
        String string = "No devices found!";
        String string2 = "To configure K9LegacyBot, please: \n   1. Attach a LegacyModuleController, \n       with \n       a. MotorController in port 0, with a \n         motor in port 1 and port 2 \n       b. ServoController in port 1, with a \n         servo in port 1 and port 6 \n      c. IR seeker in port 2\n      d. Light sensor in port 3 \n   2. Press the K9LegacyBot button\n \nTo configure K9USBBot, please: \n   1. Attach a USBMotorController, with a \n       motor in port 1 and port 2 \n    2. USBServoController in port 1, with a \n      servo in port 1 and port 6 \n   3. LegacyModule, with \n      a. IR seeker in port 2\n      b. Light sensor in port 3 \n   4. Press the K9USBBot button";
        this.g.setOrangeText(string, string2, R.id.autoconfigure_info, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
    }

    private void b() {
        String string = "Wrong devices found!";
        String string2 = "Found: \n" + this.scannedDevices.values() + "\n" + "Required: \n" + "   1. LegacyModuleController, with \n " + "      a. MotorController in port 0, with a \n" + "          motor in port 1 and port 2 \n " + "      b. ServoController in port 1, with a \n" + "          servo in port 1 and port 6 \n" + "       c. IR seeker in port 2\n" + "       d. Light sensor in port 3 ";
        this.g.setOrangeText(string, string2, R.id.autoconfigure_info, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
    }

    private void c() {
        String string = "Wrong devices found!";
        String string2 = "Found: \n" + this.scannedDevices.values() + "\n" + "Required: \n" + "   1. USBMotorController with a \n" + "      motor in port 1 and port 2 \n " + "   2. USBServoController with a \n" + "      servo in port 1 and port 6 \n" + "   3. LegacyModuleController, with \n " + "       a. IR seeker in port 2\n" + "       b. Light sensor in port 3 ";
        this.g.setOrangeText(string, string2, R.id.autoconfigure_info, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
    }

    private void d() {
        String string = "Already configured!";
        String string2 = "";
        this.g.setOrangeText(string, string2, R.id.autoconfigure_info, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
    }

    private boolean e() {
        boolean bl = true;
        boolean bl2 = true;
        boolean bl3 = true;
        for (Map.Entry<SerialNumber, DeviceManager.DeviceType> entry : this.entries) {
            DeviceManager.DeviceType deviceType = entry.getValue();
            if (deviceType == DeviceManager.DeviceType.MODERN_ROBOTICS_USB_LEGACY_MODULE && bl) {
                this.a(entry.getKey(), "sensors");
                bl = false;
            }
            if (deviceType == DeviceManager.DeviceType.MODERN_ROBOTICS_USB_DC_MOTOR_CONTROLLER && bl2) {
                this.a(entry.getKey(), "motor_1", "motor_2", "wheels");
                bl2 = false;
            }
            if (deviceType != DeviceManager.DeviceType.MODERN_ROBOTICS_USB_SERVO_CONTROLLER || !bl3) continue;
            this.a(entry.getKey(), this.f(), "servos");
            bl3 = false;
        }
        if (bl || bl2 || bl3) {
            return false;
        }
        LinearLayout linearLayout = (LinearLayout)this.findViewById(R.id.autoconfigure_info);
        linearLayout.removeAllViews();
        return true;
    }

    private ArrayList<String> f() {
        ArrayList<String> arrayList = new ArrayList<String>();
        arrayList.add("servo_1");
        arrayList.add("NO DEVICE ATTACHED");
        arrayList.add("NO DEVICE ATTACHED");
        arrayList.add("NO DEVICE ATTACHED");
        arrayList.add("NO DEVICE ATTACHED");
        arrayList.add("servo_6");
        return arrayList;
    }

    private void a(SerialNumber serialNumber, String string) {
        LegacyModuleControllerConfiguration legacyModuleControllerConfiguration = (LegacyModuleControllerConfiguration)this.e.get((Object)serialNumber);
        legacyModuleControllerConfiguration.setName(string);
        DeviceConfiguration deviceConfiguration = this.a(DeviceConfiguration.ConfigurationType.IR_SEEKER, "ir_seeker", 2);
        DeviceConfiguration deviceConfiguration2 = this.a(DeviceConfiguration.ConfigurationType.LIGHT_SENSOR, "light_sensor", 3);
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < 6; ++i) {
            if (i == 2) {
                arrayList.add(deviceConfiguration);
            }
            if (i == 3) {
                arrayList.add(deviceConfiguration2);
                continue;
            }
            DeviceConfiguration deviceConfiguration3 = new DeviceConfiguration(i);
            arrayList.add(deviceConfiguration3);
        }
        legacyModuleControllerConfiguration.addDevices(arrayList);
    }

    private boolean g() {
        boolean bl = true;
        for (Map.Entry<SerialNumber, DeviceManager.DeviceType> entry : this.entries) {
            DeviceManager.DeviceType deviceType = entry.getValue();
            if (deviceType != DeviceManager.DeviceType.MODERN_ROBOTICS_USB_LEGACY_MODULE || !bl) continue;
            this.b(entry.getKey(), "devices");
            bl = false;
        }
        if (bl) {
            return false;
        }
        LinearLayout linearLayout = (LinearLayout)this.findViewById(R.id.autoconfigure_info);
        linearLayout.removeAllViews();
        return true;
    }

    private void b(SerialNumber serialNumber, String string) {
        LegacyModuleControllerConfiguration legacyModuleControllerConfiguration = (LegacyModuleControllerConfiguration)this.e.get((Object)serialNumber);
        legacyModuleControllerConfiguration.setName(string);
        MotorControllerConfiguration motorControllerConfiguration = this.a(ControllerConfiguration.NO_SERIAL_NUMBER, "motor_1", "motor_2", "wheels");
        motorControllerConfiguration.setPort(0);
        ArrayList<String> arrayList = this.f();
        ServoControllerConfiguration servoControllerConfiguration = this.a(ControllerConfiguration.NO_SERIAL_NUMBER, arrayList, "servos");
        servoControllerConfiguration.setPort(1);
        DeviceConfiguration deviceConfiguration = this.a(DeviceConfiguration.ConfigurationType.IR_SEEKER, "ir_seeker", 2);
        DeviceConfiguration deviceConfiguration2 = this.a(DeviceConfiguration.ConfigurationType.LIGHT_SENSOR, "light_sensor", 3);
        ArrayList<Object> arrayList2 = new ArrayList<Object>();
        arrayList2.add((Object)motorControllerConfiguration);
        arrayList2.add((Object)servoControllerConfiguration);
        arrayList2.add((Object)deviceConfiguration);
        arrayList2.add((Object)deviceConfiguration2);
        for (int i = 4; i < 6; ++i) {
            DeviceConfiguration deviceConfiguration3 = new DeviceConfiguration(i);
            arrayList2.add((Object)deviceConfiguration3);
        }
        legacyModuleControllerConfiguration.addDevices(arrayList2);
    }

    private DeviceConfiguration a(DeviceConfiguration.ConfigurationType configurationType, String string, int n) {
        DeviceConfiguration deviceConfiguration = new DeviceConfiguration(n, configurationType, string, true);
        return deviceConfiguration;
    }

    private MotorControllerConfiguration a(SerialNumber serialNumber, String string, String string2, String string3) {
        MotorControllerConfiguration motorControllerConfiguration = !serialNumber.equals((Object)ControllerConfiguration.NO_SERIAL_NUMBER) ? (MotorControllerConfiguration)this.e.get((Object)serialNumber) : new MotorControllerConfiguration();
        motorControllerConfiguration.setName(string3);
        ArrayList<MotorConfiguration> arrayList = new ArrayList<MotorConfiguration>();
        MotorConfiguration motorConfiguration = new MotorConfiguration(1, string, true);
        MotorConfiguration motorConfiguration2 = new MotorConfiguration(2, string2, true);
        arrayList.add(motorConfiguration);
        arrayList.add(motorConfiguration2);
        motorControllerConfiguration.addMotors(arrayList);
        return motorControllerConfiguration;
    }

    private ServoControllerConfiguration a(SerialNumber serialNumber, ArrayList<String> arrayList, String string) {
        ServoControllerConfiguration servoControllerConfiguration = !serialNumber.equals((Object)ControllerConfiguration.NO_SERIAL_NUMBER) ? (ServoControllerConfiguration)this.e.get((Object)serialNumber) : new ServoControllerConfiguration();
        servoControllerConfiguration.setName(string);
        ArrayList<ServoConfiguration> arrayList2 = new ArrayList<ServoConfiguration>();
        for (int i = 1; i <= 6; ++i) {
            String string2 = arrayList.get(i - 1);
            boolean bl = true;
            if (string2.equals("NO DEVICE ATTACHED")) {
                bl = false;
            }
            ServoConfiguration servoConfiguration = new ServoConfiguration(i, string2, bl);
            arrayList2.add(servoConfiguration);
        }
        servoControllerConfiguration.addServos(arrayList2);
        return servoControllerConfiguration;
    }

}

