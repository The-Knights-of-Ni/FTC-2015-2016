/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Activity
 *  android.app.AlertDialog
 *  android.app.AlertDialog$Builder
 *  android.content.Context
 *  android.content.DialogInterface
 *  android.content.DialogInterface$OnClickListener
 *  android.content.Intent
 *  android.content.SharedPreferences
 *  android.os.Bundle
 *  android.preference.PreferenceManager
 *  android.text.Editable
 *  android.view.View
 *  android.view.View$OnClickListener
 *  android.widget.AdapterView
 *  android.widget.AdapterView$OnItemClickListener
 *  android.widget.Button
 *  android.widget.EditText
 *  android.widget.LinearLayout
 *  android.widget.ListAdapter
 *  android.widget.ListView
 *  android.widget.TextView
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
 *  com.qualcomm.robotcore.hardware.configuration.DeviceInfoAdapter
 *  com.qualcomm.robotcore.hardware.configuration.DeviceInterfaceModuleConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler
 *  com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.Utility
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.Util
 */
package com.qualcomm.ftccommon.configuration;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.text.Editable;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.TextView;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.R;
import com.qualcomm.ftccommon.configuration.EditDeviceInterfaceModuleActivity;
import com.qualcomm.ftccommon.configuration.EditLegacyModuleControllerActivity;
import com.qualcomm.ftccommon.configuration.EditMotorControllerActivity;
import com.qualcomm.ftccommon.configuration.EditServoControllerActivity;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceInfoAdapter;
import com.qualcomm.robotcore.hardware.configuration.DeviceInterfaceModuleConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.Util;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class FtcConfigurationActivity
extends Activity {
    private Thread d;
    private Map<SerialNumber, ControllerConfiguration> e = new HashMap<SerialNumber, ControllerConfiguration>();
    private Context f;
    private DeviceManager g;
    private Button h;
    private String i = "No current file!";
    protected Map<SerialNumber, DeviceManager.DeviceType> scannedDevices = new HashMap<SerialNumber, DeviceManager.DeviceType>();
    protected Set<Map.Entry<SerialNumber, DeviceManager.DeviceType>> scannedEntries = new HashSet<Map.Entry<SerialNumber, DeviceManager.DeviceType>>();
    protected SharedPreferences preferences;
    private Utility j;
    DialogInterface.OnClickListener a;
    DialogInterface.OnClickListener b;
    DialogInterface.OnClickListener c;

    public FtcConfigurationActivity() {
        this.a = new DialogInterface.OnClickListener(){

            public void onClick(DialogInterface dialog, int button) {
            }
        };
        this.b = new DialogInterface.OnClickListener(){

            public void onClick(DialogInterface dialog, int button) {
                FtcConfigurationActivity.this.j.saveToPreferences(FtcConfigurationActivity.this.i.substring(7).trim(), R.string.pref_hardware_config_filename);
                FtcConfigurationActivity.this.finish();
            }
        };
        this.c = new DialogInterface.OnClickListener(){

            public void onClick(DialogInterface dialog, int button) {
            }
        };
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.setContentView(R.layout.activity_ftc_configuration);
        RobotLog.writeLogcatToDisk((Context)this, (int)1024);
        this.f = this;
        this.j = new Utility((Activity)this);
        this.h = (Button)this.findViewById(R.id.scanButton);
        this.a();
        try {
            this.g = new HardwareDeviceManager(this.f, null);
        }
        catch (RobotCoreException var2_2) {
            this.j.complainToast("Failed to open the Device Manager", this.f);
            DbgLog.error("Failed to open deviceManager: " + var2_2.toString());
            DbgLog.logStacktrace((Exception)var2_2);
        }
        this.preferences = PreferenceManager.getDefaultSharedPreferences((Context)this);
    }

    private void a() {
        Button button = (Button)this.findViewById(R.id.devices_holder).findViewById(R.id.info_btn);
        button.setOnClickListener(new View.OnClickListener(){

            public void onClick(View view) {
                AlertDialog.Builder builder = FtcConfigurationActivity.this.j.buildBuilder("Devices", "These are the devices discovered by the Hardware Wizard. You can click on the name of each device to edit the information relating to that device. Make sure each device has a unique name. The names should match what is in the Op mode code. Scroll down to see more devices if there are any.");
                builder.setPositiveButton((CharSequence)"Ok", FtcConfigurationActivity.this.a);
                AlertDialog alertDialog = builder.create();
                alertDialog.show();
                TextView textView = (TextView)alertDialog.findViewById(16908299);
                textView.setTextSize(14.0f);
            }
        });
        Button button2 = (Button)this.findViewById(R.id.save_holder).findViewById(R.id.info_btn);
        button2.setOnClickListener(new View.OnClickListener(){

            public void onClick(View view) {
                AlertDialog.Builder builder = FtcConfigurationActivity.this.j.buildBuilder("Save Configuration", "Clicking the save button will create an xml file in: \n      /sdcard/FIRST/  \nThis file will be used to initialize the robot.");
                builder.setPositiveButton((CharSequence)"Ok", FtcConfigurationActivity.this.a);
                AlertDialog alertDialog = builder.create();
                alertDialog.show();
                TextView textView = (TextView)alertDialog.findViewById(16908299);
                textView.setTextSize(14.0f);
            }
        });
    }

    protected void onStart() {
        super.onStart();
        this.i = this.j.getFilenameFromPrefs(R.string.pref_hardware_config_filename, "No current file!");
        if (this.i.equalsIgnoreCase("No current file!")) {
            this.j.createConfigFolder();
        }
        this.j.updateHeader("No current file!", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
        this.e();
        if (!this.i.toLowerCase().contains((CharSequence)"Unsaved".toLowerCase())) {
            this.d();
        }
        this.h.setOnClickListener(new View.OnClickListener(){

            public void onClick(View view) {
                FtcConfigurationActivity.this.d = new Thread(new Runnable(){

                    @Override
                    public void run() {
                        Util.logThreadLifeCycle((String)"Scanning USB Bus", (Runnable)new Runnable(){

                            @Override
                            public void run() {
                                try {
                                    DbgLog.msg("Scanning USB bus");
                                    FtcConfigurationActivity.this.scannedDevices = FtcConfigurationActivity.this.g.scanForUsbDevices();
                                }
                                catch (RobotCoreException var1_1) {
                                    DbgLog.error("Device scan failed: " + var1_1.toString());
                                }
                                FtcConfigurationActivity.this.runOnUiThread(new Runnable(){

                                    @Override
                                    public void run() {
                                        FtcConfigurationActivity.this.j.resetCount();
                                        FtcConfigurationActivity.this.g();
                                        FtcConfigurationActivity.this.i();
                                        FtcConfigurationActivity.this.j.updateHeader(FtcConfigurationActivity.this.i, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                                        FtcConfigurationActivity.this.scannedEntries = FtcConfigurationActivity.this.scannedDevices.entrySet();
                                        FtcConfigurationActivity.this.e = FtcConfigurationActivity.this.b();
                                        FtcConfigurationActivity.this.h();
                                        FtcConfigurationActivity.this.f();
                                    }
                                });
                            }

                        });
                    }

                });
                FtcConfigurationActivity.this.c();
            }

        });
    }

    private HashMap<SerialNumber, ControllerConfiguration> b() {
        HashMap<SerialNumber, ControllerConfiguration> hashMap = new HashMap<SerialNumber, ControllerConfiguration>();
        for (Map.Entry<SerialNumber, DeviceManager.DeviceType> entry : this.scannedEntries) {
            SerialNumber serialNumber = entry.getKey();
            if (this.e.containsKey((Object)serialNumber)) {
                hashMap.put(serialNumber, this.e.get((Object)serialNumber));
                continue;
            }
            switch (entry.getValue()) {
                case MODERN_ROBOTICS_USB_DC_MOTOR_CONTROLLER: {
                    hashMap.put(serialNumber, (ControllerConfiguration)this.j.buildMotorController(serialNumber));
                    break;
                }
                case MODERN_ROBOTICS_USB_SERVO_CONTROLLER: {
                    hashMap.put(serialNumber, (ControllerConfiguration)this.j.buildServoController(serialNumber));
                    break;
                }
                case MODERN_ROBOTICS_USB_LEGACY_MODULE: {
                    hashMap.put(serialNumber, (ControllerConfiguration)this.j.buildLegacyModule(serialNumber));
                    break;
                }
                case MODERN_ROBOTICS_USB_DEVICE_INTERFACE_MODULE: {
                    hashMap.put(serialNumber, (ControllerConfiguration)this.j.buildDeviceInterfaceModule(serialNumber));
                }
            }
        }
        return hashMap;
    }

    private void c() {
        if (this.i.toLowerCase().contains((CharSequence)"Unsaved".toLowerCase())) {
            String string = "If you scan, your current unsaved changes will be lost.";
            EditText editText = new EditText(this.f);
            editText.setEnabled(false);
            editText.setText((CharSequence)"");
            AlertDialog.Builder builder = this.j.buildBuilder("Unsaved changes", string);
            builder.setView((View)editText);
            DialogInterface.OnClickListener onClickListener = new DialogInterface.OnClickListener(){

                public void onClick(DialogInterface dialog, int button) {
                    FtcConfigurationActivity.this.d.start();
                }
            };
            builder.setPositiveButton((CharSequence)"Ok", onClickListener);
            builder.setNegativeButton((CharSequence)"Cancel", this.c);
            builder.show();
        } else {
            this.d.start();
        }
    }

    private void d() {
        ReadXMLFileHandler readXMLFileHandler = new ReadXMLFileHandler(this.f);
        if (this.i.equalsIgnoreCase("No current file!")) {
            return;
        }
        FileInputStream fileInputStream = null;
        try {
            fileInputStream = new FileInputStream(Utility.CONFIG_FILES_DIR + this.i + ".xml");
            ArrayList arrayList = (ArrayList)readXMLFileHandler.parse((InputStream)fileInputStream);
            this.a(arrayList);
            this.h();
            this.f();
        }
        catch (RobotCoreException var3_4) {
            RobotLog.e((String)"Error parsing XML file");
            RobotLog.logStacktrace((RobotCoreException)var3_4);
            this.j.complainToast("Error parsing XML file: " + this.i, this.f);
            return;
        }
        catch (FileNotFoundException var3_5) {
            DbgLog.error("File was not found: " + this.i);
            DbgLog.logStacktrace(var3_5);
            this.j.complainToast("That file was not found: " + this.i, this.f);
            return;
        }
    }

    private void e() {
        if (this.e.size() == 0) {
            String string = "Scan to find devices.";
            String string2 = "In order to find devices: \n    1. Attach a robot \n    2. Press the 'Scan' button";
            this.j.setOrangeText(string, string2, R.id.empty_devicelist, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
            this.g();
        } else {
            LinearLayout linearLayout = (LinearLayout)this.findViewById(R.id.empty_devicelist);
            linearLayout.removeAllViews();
            linearLayout.setVisibility(8);
        }
    }

    private void f() {
        if (this.e.size() == 0) {
            String string = "No devices found!";
            String string2 = "In order to find devices: \n    1. Attach a robot \n    2. Press the 'Scan' button";
            this.j.setOrangeText(string, string2, R.id.empty_devicelist, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
            this.g();
        } else {
            LinearLayout linearLayout = (LinearLayout)this.findViewById(R.id.empty_devicelist);
            linearLayout.removeAllViews();
            linearLayout.setVisibility(8);
        }
    }

    private void a(String string) {
        String string2 = "Found " + string;
        String string3 = "Please fix and re-save.";
        this.j.setOrangeText(string2, string3, R.id.warning_layout, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
    }

    private void g() {
        LinearLayout linearLayout = (LinearLayout)this.findViewById(R.id.warning_layout);
        linearLayout.removeAllViews();
        linearLayout.setVisibility(8);
    }

    private void h() {
        ListView listView = (ListView)this.findViewById(R.id.controllersList);
        DeviceInfoAdapter deviceInfoAdapter = new DeviceInfoAdapter((Activity)this, 17367044, this.e);
        listView.setAdapter((ListAdapter)deviceInfoAdapter);
        listView.setOnItemClickListener(new AdapterView.OnItemClickListener(){

            public void onItemClick(AdapterView<?> adapterView, View v, int pos, long arg3) {
                Intent intent;
                int n;
                ControllerConfiguration controllerConfiguration = (ControllerConfiguration)adapterView.getItemAtPosition(pos);
                DeviceConfiguration.ConfigurationType configurationType = controllerConfiguration.getType();
                if (configurationType.equals((Object)DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER)) {
                    n = 1;
                    intent = new Intent(FtcConfigurationActivity.this.f, (Class)EditMotorControllerActivity.class);
                    intent.putExtra("EDIT_MOTOR_CONTROLLER_CONFIG", (Serializable)controllerConfiguration);
                    intent.putExtra("requestCode", n);
                    FtcConfigurationActivity.this.setResult(-1, intent);
                    FtcConfigurationActivity.this.startActivityForResult(intent, n);
                }
                if (configurationType.equals((Object)DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER)) {
                    n = 2;
                    intent = new Intent(FtcConfigurationActivity.this.f, (Class)EditServoControllerActivity.class);
                    intent.putExtra("Edit Servo ControllerConfiguration Activity", (Serializable)controllerConfiguration);
                    intent.putExtra("requestCode", n);
                    FtcConfigurationActivity.this.setResult(-1, intent);
                    FtcConfigurationActivity.this.startActivityForResult(intent, n);
                }
                if (configurationType.equals((Object)DeviceConfiguration.ConfigurationType.LEGACY_MODULE_CONTROLLER)) {
                    n = 3;
                    intent = new Intent(FtcConfigurationActivity.this.f, (Class)EditLegacyModuleControllerActivity.class);
                    intent.putExtra("EDIT_LEGACY_CONFIG", (Serializable)controllerConfiguration);
                    intent.putExtra("requestCode", n);
                    FtcConfigurationActivity.this.setResult(-1, intent);
                    FtcConfigurationActivity.this.startActivityForResult(intent, n);
                }
                if (configurationType.equals((Object)DeviceConfiguration.ConfigurationType.DEVICE_INTERFACE_MODULE)) {
                    n = 4;
                    intent = new Intent(FtcConfigurationActivity.this.f, (Class)EditDeviceInterfaceModuleActivity.class);
                    intent.putExtra("EDIT_DEVICE_INTERFACE_MODULE_CONFIG", (Serializable)controllerConfiguration);
                    intent.putExtra("requestCode", n);
                    FtcConfigurationActivity.this.setResult(-1, intent);
                    FtcConfigurationActivity.this.startActivityForResult(intent, n);
                }
            }
        });
    }

    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == 0) {
            return;
        }
        Serializable serializable = null;
        if (requestCode == 1) {
            serializable = data.getSerializableExtra("EDIT_MOTOR_CONTROLLER_CONFIG");
        } else if (requestCode == 2) {
            serializable = data.getSerializableExtra("Edit Servo ControllerConfiguration Activity");
        } else if (requestCode == 3) {
            serializable = data.getSerializableExtra("EDIT_LEGACY_CONFIG");
        } else if (requestCode == 4) {
            serializable = data.getSerializableExtra("EDIT_DEVICE_INTERFACE_MODULE_CONFIG");
        }
        if (serializable != null) {
            ControllerConfiguration controllerConfiguration = (ControllerConfiguration)serializable;
            this.scannedDevices.put(controllerConfiguration.getSerialNumber(), controllerConfiguration.configTypeToDeviceType(controllerConfiguration.getType()));
            this.e.put(controllerConfiguration.getSerialNumber(), controllerConfiguration);
            this.h();
            this.i();
        } else {
            DbgLog.error("Received Result with an incorrect request code: " + String.valueOf(requestCode));
        }
    }

    private void i() {
        if (!this.i.toLowerCase().contains((CharSequence)"Unsaved".toLowerCase())) {
            String string = "Unsaved " + this.i;
            this.j.saveToPreferences(string, R.string.pref_hardware_config_filename);
            this.i = string;
        }
    }

    protected void onDestroy() {
        super.onDestroy();
        this.j.resetCount();
    }

    public void onBackPressed() {
        if (this.i.toLowerCase().contains((CharSequence)"Unsaved".toLowerCase())) {
            boolean bl = this.j.writeXML(this.e);
            if (bl) {
                return;
            }
            String string = "Please save your file before exiting the Hardware Wizard! \n If you click 'Cancel' your changes will be lost.";
            final EditText editText = new EditText((Context)this);
            String string2 = this.j.prepareFilename(this.i);
            editText.setText((CharSequence)string2);
            AlertDialog.Builder builder = this.j.buildBuilder("Unsaved changes", string);
            builder.setView((View)editText);
            DialogInterface.OnClickListener onClickListener = new DialogInterface.OnClickListener(){

                public void onClick(DialogInterface dialog, int button) {
                    String string = editText.getText().toString() + ".xml";
                    if ((string = string.trim()).equals(".xml")) {
                        FtcConfigurationActivity.this.j.complainToast("File not saved: Please entire a filename.", FtcConfigurationActivity.this.f);
                        return;
                    }
                    try {
                        FtcConfigurationActivity.this.j.writeToFile(string);
                    }
                    catch (RobotCoreException var4_4) {
                        FtcConfigurationActivity.this.j.complainToast(var4_4.getMessage(), FtcConfigurationActivity.this.f);
                        DbgLog.error(var4_4.getMessage());
                        return;
                    }
                    catch (IOException var4_5) {
                        FtcConfigurationActivity.this.a(var4_5.getMessage());
                        DbgLog.error(var4_5.getMessage());
                        return;
                    }
                    FtcConfigurationActivity.this.g();
                    FtcConfigurationActivity.this.j.saveToPreferences(editText.getText().toString(), R.string.pref_hardware_config_filename);
                    FtcConfigurationActivity.this.i = editText.getText().toString();
                    FtcConfigurationActivity.this.j.updateHeader("robot_config", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                    FtcConfigurationActivity.this.j.confirmSave();
                    FtcConfigurationActivity.this.finish();
                }
            };
            builder.setPositiveButton((CharSequence)"Ok", onClickListener);
            builder.setNegativeButton((CharSequence)"Cancel", this.b);
            builder.show();
        } else {
            super.onBackPressed();
        }
    }

    public void saveConfiguration(View v) {
        boolean bl = this.j.writeXML(this.e);
        if (bl) {
            return;
        }
        String string = "Please enter the file name";
        final EditText editText = new EditText((Context)this);
        String string2 = this.j.prepareFilename(this.i);
        editText.setText((CharSequence)string2);
        AlertDialog.Builder builder = this.j.buildBuilder("Enter file name", string);
        builder.setView((View)editText);
        DialogInterface.OnClickListener onClickListener = new DialogInterface.OnClickListener(){

            public void onClick(DialogInterface dialog, int button) {
                String string = editText.getText().toString() + ".xml";
                if ((string = string.trim()).equals(".xml")) {
                    FtcConfigurationActivity.this.j.complainToast("File not saved: Please entire a filename.", FtcConfigurationActivity.this.f);
                    return;
                }
                try {
                    FtcConfigurationActivity.this.j.writeToFile(string);
                }
                catch (RobotCoreException var4_4) {
                    FtcConfigurationActivity.this.j.complainToast(var4_4.getMessage(), FtcConfigurationActivity.this.f);
                    DbgLog.error(var4_4.getMessage());
                    return;
                }
                catch (IOException var4_5) {
                    FtcConfigurationActivity.this.a(var4_5.getMessage());
                    DbgLog.error(var4_5.getMessage());
                    return;
                }
                FtcConfigurationActivity.this.g();
                FtcConfigurationActivity.this.j.saveToPreferences(editText.getText().toString(), R.string.pref_hardware_config_filename);
                FtcConfigurationActivity.this.i = editText.getText().toString();
                FtcConfigurationActivity.this.j.updateHeader("robot_config", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                FtcConfigurationActivity.this.j.confirmSave();
            }
        };
        builder.setPositiveButton((CharSequence)"Ok", onClickListener);
        builder.setNegativeButton((CharSequence)"Cancel", this.c);
        builder.show();
    }

    private void a(ArrayList<ControllerConfiguration> arrayList) {
        this.e = new HashMap<SerialNumber, ControllerConfiguration>();
        for (ControllerConfiguration controllerConfiguration : arrayList) {
            this.e.put(controllerConfiguration.getSerialNumber(), controllerConfiguration);
        }
    }

}

