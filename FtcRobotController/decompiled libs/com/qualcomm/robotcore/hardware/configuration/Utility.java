/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Activity
 *  android.app.AlertDialog
 *  android.app.AlertDialog$Builder
 *  android.content.Context
 *  android.content.SharedPreferences
 *  android.content.SharedPreferences$Editor
 *  android.graphics.Color
 *  android.os.Environment
 *  android.preference.PreferenceManager
 *  android.view.LayoutInflater
 *  android.view.View
 *  android.view.ViewGroup
 *  android.widget.LinearLayout
 *  android.widget.TextView
 *  android.widget.Toast
 */
package com.qualcomm.robotcore.hardware.configuration;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceInterfaceModuleConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.WriteXMLFileHandler;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Utility {
    public static final String AUTOCONFIGURE_K9LEGACYBOT = "K9LegacyBot";
    public static final String AUTOCONFIGURE_K9USBBOT = "K9USBBot";
    public static final String CONFIG_FILES_DIR = Environment.getExternalStorageDirectory() + "/FIRST/";
    public static final String DEFAULT_ROBOT_CONFIG = "robot_config";
    public static final String FILE_EXT = ".xml";
    public static final String DEFAULT_ROBOT_CONFIG_FILENAME = "robot_config.xml";
    public static final String NO_FILE = "No current file!";
    public static final String UNSAVED = "Unsaved";
    private Activity a;
    private SharedPreferences b;
    private static int c = 1;
    private WriteXMLFileHandler d;
    private String e;

    public Utility(Activity activity) {
        this.a = activity;
        this.b = PreferenceManager.getDefaultSharedPreferences((Context)activity);
        this.d = new WriteXMLFileHandler((Context)activity);
    }

    public void createConfigFolder() {
        File file = new File(CONFIG_FILES_DIR);
        boolean bl = true;
        if (!file.exists()) {
            bl = file.mkdir();
        }
        if (!bl) {
            RobotLog.e("Can't create the Robot Config Files directory!");
            this.complainToast("Can't create the Robot Config Files directory!", (Context)this.a);
        }
    }

    public ArrayList<String> getXMLFiles() {
        File file = new File(CONFIG_FILES_DIR);
        File[] arrfile = file.listFiles();
        if (arrfile == null) {
            RobotLog.i("robotConfigFiles directory is empty");
            return new ArrayList<String>();
        }
        ArrayList<String> arrayList = new ArrayList<String>();
        for (File file2 : arrfile) {
            if (!file2.isFile()) continue;
            String string = file2.getName();
            Pattern pattern = Pattern.compile("(?i).xml");
            if (!pattern.matcher((CharSequence)string).find()) continue;
            String string2 = string.replaceFirst("[.][^.]+$", "");
            arrayList.add(string2);
        }
        return arrayList;
    }

    public boolean writeXML(Map<SerialNumber, ControllerConfiguration> deviceControllers) {
        block2 : {
            ArrayList<ControllerConfiguration> arrayList = new ArrayList<ControllerConfiguration>();
            arrayList.addAll(deviceControllers.values());
            try {
                this.e = this.d.writeXml(arrayList);
            }
            catch (RuntimeException var3_3) {
                if (!var3_3.getMessage().contains((CharSequence)"Duplicate name")) break block2;
                this.complainToast("Found " + var3_3.getMessage(), (Context)this.a);
                RobotLog.e("Found " + var3_3.getMessage());
                return true;
            }
        }
        return false;
    }

    public void writeToFile(String filename) throws RobotCoreException, IOException {
        this.d.writeToFile(this.e, CONFIG_FILES_DIR, filename);
    }

    public String getOutput() {
        return this.e;
    }

    public void complainToast(String msg, Context context) {
        Toast toast = Toast.makeText((Context)context, (CharSequence)msg, (int)0);
        toast.setGravity(17, 0, 0);
        TextView textView = (TextView)toast.getView().findViewById(16908299);
        textView.setTextColor(-1);
        textView.setTextSize(18.0f);
        toast.show();
    }

    public void createLists(Set<Map.Entry<SerialNumber, DeviceManager.DeviceType>> entries, Map<SerialNumber, ControllerConfiguration> deviceControllers) {
        for (Map.Entry<SerialNumber, DeviceManager.DeviceType> entry : entries) {
            DeviceManager.DeviceType deviceType = entry.getValue();
            switch (deviceType) {
                case MODERN_ROBOTICS_USB_DC_MOTOR_CONTROLLER: {
                    deviceControllers.put(entry.getKey(), this.buildMotorController(entry.getKey()));
                    break;
                }
                case MODERN_ROBOTICS_USB_SERVO_CONTROLLER: {
                    deviceControllers.put(entry.getKey(), this.buildServoController(entry.getKey()));
                    break;
                }
                case MODERN_ROBOTICS_USB_LEGACY_MODULE: {
                    deviceControllers.put(entry.getKey(), this.buildLegacyModule(entry.getKey()));
                    break;
                }
                case MODERN_ROBOTICS_USB_DEVICE_INTERFACE_MODULE: {
                    DeviceInterfaceModuleConfiguration deviceInterfaceModuleConfiguration = this.buildDeviceInterfaceModule(entry.getKey());
                    deviceControllers.put(entry.getKey(), deviceInterfaceModuleConfiguration);
                    break;
                }
            }
        }
    }

    public DeviceInterfaceModuleConfiguration buildDeviceInterfaceModule(SerialNumber serialNumber) {
        DeviceInterfaceModuleConfiguration deviceInterfaceModuleConfiguration = new DeviceInterfaceModuleConfiguration("Device Interface Module " + c, serialNumber);
        deviceInterfaceModuleConfiguration.setPwmDevices(this.createPWMList());
        deviceInterfaceModuleConfiguration.setI2cDevices(this.createI2CList());
        deviceInterfaceModuleConfiguration.setAnalogInputDevices(this.createAnalogInputList());
        deviceInterfaceModuleConfiguration.setDigitalDevices(this.createDigitalList());
        deviceInterfaceModuleConfiguration.setAnalogOutputDevices(this.createAnalogOutputList());
        ++c;
        return deviceInterfaceModuleConfiguration;
    }

    public LegacyModuleControllerConfiguration buildLegacyModule(SerialNumber serialNumber) {
        ArrayList<DeviceConfiguration> arrayList = this.createLegacyModuleList();
        LegacyModuleControllerConfiguration legacyModuleControllerConfiguration = new LegacyModuleControllerConfiguration("Legacy Module " + c, arrayList, serialNumber);
        ++c;
        return legacyModuleControllerConfiguration;
    }

    public ServoControllerConfiguration buildServoController(SerialNumber serialNumber) {
        ArrayList<DeviceConfiguration> arrayList = this.createServoList();
        ServoControllerConfiguration servoControllerConfiguration = new ServoControllerConfiguration("Servo Controller " + c, arrayList, serialNumber);
        ++c;
        return servoControllerConfiguration;
    }

    public MotorControllerConfiguration buildMotorController(SerialNumber serialNumber) {
        ArrayList<DeviceConfiguration> arrayList = this.createMotorList();
        MotorControllerConfiguration motorControllerConfiguration = new MotorControllerConfiguration("Motor Controller " + c, arrayList, serialNumber);
        ++c;
        return motorControllerConfiguration;
    }

    public ArrayList<DeviceConfiguration> createMotorList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        MotorConfiguration motorConfiguration = new MotorConfiguration(1);
        arrayList.add(motorConfiguration);
        MotorConfiguration motorConfiguration2 = new MotorConfiguration(2);
        arrayList.add(motorConfiguration2);
        return arrayList;
    }

    public ArrayList<DeviceConfiguration> createServoList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 1; i <= 6; ++i) {
            ServoConfiguration servoConfiguration = new ServoConfiguration(i);
            arrayList.add(servoConfiguration);
        }
        return arrayList;
    }

    public ArrayList<DeviceConfiguration> createLegacyModuleList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < 6; ++i) {
            DeviceConfiguration deviceConfiguration = new DeviceConfiguration(i, DeviceConfiguration.ConfigurationType.NOTHING);
            arrayList.add(deviceConfiguration);
        }
        return arrayList;
    }

    public ArrayList<DeviceConfiguration> createPWMList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < 2; ++i) {
            DeviceConfiguration deviceConfiguration = new DeviceConfiguration(i, DeviceConfiguration.ConfigurationType.PULSE_WIDTH_DEVICE);
            arrayList.add(deviceConfiguration);
        }
        return arrayList;
    }

    public ArrayList<DeviceConfiguration> createI2CList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < 6; ++i) {
            DeviceConfiguration deviceConfiguration = new DeviceConfiguration(i, DeviceConfiguration.ConfigurationType.I2C_DEVICE);
            arrayList.add(deviceConfiguration);
        }
        return arrayList;
    }

    public ArrayList<DeviceConfiguration> createAnalogInputList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < 8; ++i) {
            DeviceConfiguration deviceConfiguration = new DeviceConfiguration(i, DeviceConfiguration.ConfigurationType.ANALOG_INPUT);
            arrayList.add(deviceConfiguration);
        }
        return arrayList;
    }

    public ArrayList<DeviceConfiguration> createDigitalList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < 8; ++i) {
            DeviceConfiguration deviceConfiguration = new DeviceConfiguration(i, DeviceConfiguration.ConfigurationType.DIGITAL_DEVICE);
            arrayList.add(deviceConfiguration);
        }
        return arrayList;
    }

    public ArrayList<DeviceConfiguration> createAnalogOutputList() {
        ArrayList<DeviceConfiguration> arrayList = new ArrayList<DeviceConfiguration>();
        for (int i = 0; i < 2; ++i) {
            DeviceConfiguration deviceConfiguration = new DeviceConfiguration(i, DeviceConfiguration.ConfigurationType.ANALOG_OUTPUT);
            arrayList.add(deviceConfiguration);
        }
        return arrayList;
    }

    public void updateHeader(String default_name, int pref_hardware_config_filename_id, int fileTextView, int header_id) {
        String string = this.b.getString(this.a.getString(pref_hardware_config_filename_id), default_name);
        String string2 = string.replaceFirst("[.][^.]+$", "");
        TextView textView = (TextView)this.a.findViewById(fileTextView);
        textView.setText((CharSequence)string2);
        if (string2.equalsIgnoreCase("No current file!")) {
            int n = Color.parseColor((String)"#bf0510");
            this.changeBackground(n, header_id);
        } else if (string2.toLowerCase().contains((CharSequence)"Unsaved".toLowerCase())) {
            this.changeBackground(-12303292, header_id);
        } else {
            int n = Color.parseColor((String)"#790E15");
            this.changeBackground(n, header_id);
        }
    }

    public void saveToPreferences(String filename, int pref_hardware_config_filename_id) {
        filename = filename.replaceFirst("[.][^.]+$", "");
        SharedPreferences.Editor editor = this.b.edit();
        editor.putString(this.a.getString(pref_hardware_config_filename_id), filename);
        editor.apply();
    }

    public void changeBackground(int color, int header_id) {
        LinearLayout linearLayout = (LinearLayout)this.a.findViewById(header_id);
        linearLayout.setBackgroundColor(color);
    }

    public String getFilenameFromPrefs(int pref_hardware_config_filename_id, String default_name) {
        return this.b.getString(this.a.getString(pref_hardware_config_filename_id), default_name);
    }

    public void resetCount() {
        c = 1;
    }

    public void setOrangeText(String msg0, String msg1, int info_id, int layout_id, int orange0, int orange1) {
        LinearLayout linearLayout = (LinearLayout)this.a.findViewById(info_id);
        linearLayout.setVisibility(0);
        linearLayout.removeAllViews();
        this.a.getLayoutInflater().inflate(layout_id, (ViewGroup)linearLayout, true);
        TextView textView = (TextView)linearLayout.findViewById(orange0);
        TextView textView2 = (TextView)linearLayout.findViewById(orange1);
        textView2.setGravity(3);
        textView.setText((CharSequence)msg0);
        textView2.setText((CharSequence)msg1);
    }

    public void confirmSave() {
        Toast toast = Toast.makeText((Context)this.a, (CharSequence)"Saved", (int)0);
        toast.setGravity(80, 0, 50);
        toast.show();
    }

    public AlertDialog.Builder buildBuilder(String title, String message) {
        AlertDialog.Builder builder = new AlertDialog.Builder((Context)this.a);
        builder.setTitle((CharSequence)title).setMessage((CharSequence)message);
        return builder;
    }

    public String prepareFilename(String currentFile) {
        if (currentFile.toLowerCase().contains((CharSequence)"Unsaved".toLowerCase())) {
            currentFile = currentFile.substring(7).trim();
        }
        if (currentFile.equalsIgnoreCase("No current file!")) {
            currentFile = "";
        }
        return currentFile;
    }

}

