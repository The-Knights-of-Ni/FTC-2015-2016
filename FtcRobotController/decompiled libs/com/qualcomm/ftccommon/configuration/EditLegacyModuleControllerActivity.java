/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Activity
 *  android.content.Context
 *  android.content.Intent
 *  android.os.Bundle
 *  android.preference.PreferenceManager
 *  android.text.Editable
 *  android.text.TextWatcher
 *  android.view.LayoutInflater
 *  android.view.View
 *  android.view.ViewGroup
 *  android.view.ViewParent
 *  android.widget.AdapterView
 *  android.widget.AdapterView$OnItemSelectedListener
 *  android.widget.ArrayAdapter
 *  android.widget.Button
 *  android.widget.EditText
 *  android.widget.LinearLayout
 *  android.widget.Spinner
 *  android.widget.SpinnerAdapter
 *  android.widget.TextView
 *  com.qualcomm.ftccommon.R
 *  com.qualcomm.ftccommon.R$id
 *  com.qualcomm.ftccommon.R$layout
 *  com.qualcomm.ftccommon.R$string
 *  com.qualcomm.ftccommon.R$xml
 *  com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration$ConfigurationType
 *  com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MatrixControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MotorConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.ServoConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.Utility
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.ftccommon.configuration;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewParent;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.SpinnerAdapter;
import android.widget.TextView;
import com.qualcomm.ftccommon.R;
import com.qualcomm.ftccommon.configuration.EditMatrixControllerActivity;
import com.qualcomm.ftccommon.configuration.EditMotorControllerActivity;
import com.qualcomm.ftccommon.configuration.EditServoControllerActivity;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MatrixControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class EditLegacyModuleControllerActivity
extends Activity {
    private static boolean a = false;
    private Utility b;
    private String c;
    private Context d;
    public static final String EDIT_LEGACY_CONFIG = "EDIT_LEGACY_CONFIG";
    public static final int EDIT_MOTOR_CONTROLLER_REQUEST_CODE = 101;
    public static final int EDIT_SERVO_CONTROLLER_REQUEST_CODE = 102;
    public static final int EDIT_MATRIX_CONTROLLER_REQUEST_CODE = 103;
    private LegacyModuleControllerConfiguration e;
    private EditText f;
    private ArrayList<DeviceConfiguration> g = new ArrayList();
    private View h;
    private View i;
    private View j;
    private View k;
    private View l;
    private View m;
    private AdapterView.OnItemSelectedListener n;

    public EditLegacyModuleControllerActivity() {
        this.n = new AdapterView.OnItemSelectedListener(){

            public void onItemSelected(AdapterView<?> parent, View view, int pos, long l) {
                String string = parent.getItemAtPosition(pos).toString();
                LinearLayout linearLayout = (LinearLayout)view.getParent().getParent().getParent();
                if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.NOTHING.toString())) {
                    EditLegacyModuleControllerActivity.this.a(linearLayout);
                } else {
                    EditLegacyModuleControllerActivity.this.a(linearLayout, string);
                }
            }

            public void onNothingSelected(AdapterView<?> adapterView) {
            }
        };
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.setContentView(R.layout.legacy);
        LinearLayout linearLayout = (LinearLayout)this.findViewById(R.id.linearLayout0);
        this.h = this.getLayoutInflater().inflate(R.layout.simple_device, (ViewGroup)linearLayout, true);
        TextView textView = (TextView)this.h.findViewById(R.id.portNumber);
        textView.setText((CharSequence)"0");
        LinearLayout linearLayout2 = (LinearLayout)this.findViewById(R.id.linearLayout1);
        this.i = this.getLayoutInflater().inflate(R.layout.simple_device, (ViewGroup)linearLayout2, true);
        TextView textView2 = (TextView)this.i.findViewById(R.id.portNumber);
        textView2.setText((CharSequence)"1");
        LinearLayout linearLayout3 = (LinearLayout)this.findViewById(R.id.linearLayout2);
        this.j = this.getLayoutInflater().inflate(R.layout.simple_device, (ViewGroup)linearLayout3, true);
        TextView textView3 = (TextView)this.j.findViewById(R.id.portNumber);
        textView3.setText((CharSequence)"2");
        LinearLayout linearLayout4 = (LinearLayout)this.findViewById(R.id.linearLayout3);
        this.k = this.getLayoutInflater().inflate(R.layout.simple_device, (ViewGroup)linearLayout4, true);
        TextView textView4 = (TextView)this.k.findViewById(R.id.portNumber);
        textView4.setText((CharSequence)"3");
        LinearLayout linearLayout5 = (LinearLayout)this.findViewById(R.id.linearLayout4);
        this.l = this.getLayoutInflater().inflate(R.layout.simple_device, (ViewGroup)linearLayout5, true);
        TextView textView5 = (TextView)this.l.findViewById(R.id.portNumber);
        textView5.setText((CharSequence)"4");
        LinearLayout linearLayout6 = (LinearLayout)this.findViewById(R.id.linearLayout5);
        this.m = this.getLayoutInflater().inflate(R.layout.simple_device, (ViewGroup)linearLayout6, true);
        TextView textView6 = (TextView)this.m.findViewById(R.id.portNumber);
        textView6.setText((CharSequence)"5");
        this.d = this;
        PreferenceManager.setDefaultValues((Context)this, (int)R.xml.preferences, (boolean)false);
        this.b = new Utility((Activity)this);
        RobotLog.writeLogcatToDisk((Context)this, (int)1024);
        this.f = (EditText)this.findViewById(R.id.device_interface_module_name);
    }

    protected void onStart() {
        super.onStart();
        this.b.updateHeader("No current file!", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
        this.c = this.b.getFilenameFromPrefs(R.string.pref_hardware_config_filename, "No current file!");
        Intent intent = this.getIntent();
        Serializable serializable = intent.getSerializableExtra("EDIT_LEGACY_CONFIG");
        if (serializable != null) {
            this.e = (LegacyModuleControllerConfiguration)serializable;
            this.g = (ArrayList)this.e.getDevices();
            this.f.setText((CharSequence)this.e.getName());
            this.f.addTextChangedListener((TextWatcher)new a(this));
            TextView textView = (TextView)this.findViewById(R.id.legacy_serialNumber);
            textView.setText((CharSequence)this.e.getSerialNumber().toString());
            for (int i = 0; i < this.g.size(); ++i) {
                DeviceConfiguration deviceConfiguration = this.g.get(i);
                if (a) {
                    RobotLog.e((String)("[onStart] module name: " + deviceConfiguration.getName() + ", port: " + deviceConfiguration.getPort() + ", type: " + (Object)deviceConfiguration.getType()));
                }
                this.a(this.a(i), deviceConfiguration);
            }
        }
    }

    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        Serializable serializable = null;
        if (resultCode == -1) {
            if (requestCode == 101) {
                serializable = data.getSerializableExtra("EDIT_MOTOR_CONTROLLER_CONFIG");
            } else if (requestCode == 102) {
                serializable = data.getSerializableExtra("Edit Servo ControllerConfiguration Activity");
            } else if (requestCode == 103) {
                serializable = data.getSerializableExtra("Edit Matrix ControllerConfiguration Activity");
            }
            if (serializable != null) {
                ControllerConfiguration controllerConfiguration = (ControllerConfiguration)serializable;
                this.b((DeviceConfiguration)controllerConfiguration);
                this.a(this.a(controllerConfiguration.getPort()), this.g.get(controllerConfiguration.getPort()));
                String string = this.c;
                if (!string.toLowerCase().contains((CharSequence)"Unsaved".toLowerCase())) {
                    string = "Unsaved " + this.c;
                    this.b.saveToPreferences(string, R.string.pref_hardware_config_filename);
                    this.c = string;
                }
                this.b.updateHeader("No current file!", R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
            }
        }
    }

    public void saveLegacyController(View v) {
        this.a();
    }

    private void a() {
        Intent intent = new Intent();
        this.e.setName(this.f.getText().toString());
        intent.putExtra("EDIT_LEGACY_CONFIG", (Serializable)this.e);
        this.setResult(-1, intent);
        this.finish();
    }

    public void cancelLegacyController(View v) {
        this.setResult(0, new Intent());
        this.finish();
    }

    private void a(View view, DeviceConfiguration deviceConfiguration) {
        Spinner spinner = (Spinner)view.findViewById(R.id.choiceSpinner_legacyModule);
        ArrayAdapter arrayAdapter = (ArrayAdapter)spinner.getAdapter();
        int n = arrayAdapter.getPosition((Object)deviceConfiguration.getType().toString());
        spinner.setSelection(n);
        spinner.setOnItemSelectedListener(this.n);
        String string = deviceConfiguration.getName();
        EditText editText = (EditText)view.findViewById(R.id.editTextResult_name);
        TextView textView = (TextView)view.findViewById(R.id.portNumber);
        int n2 = Integer.parseInt(textView.getText().toString());
        editText.addTextChangedListener((TextWatcher)new a(this, this.a(n2)));
        editText.setText((CharSequence)string);
        if (a) {
            RobotLog.e((String)("[populatePort] name: " + string + ", port: " + n2 + ", type: " + (Object)deviceConfiguration.getType()));
        }
    }

    private void a(LinearLayout linearLayout) {
        TextView textView = (TextView)linearLayout.findViewById(R.id.portNumber);
        int n = Integer.parseInt(textView.getText().toString());
        EditText editText = (EditText)linearLayout.findViewById(R.id.editTextResult_name);
        editText.setEnabled(false);
        editText.setText((CharSequence)"NO DEVICE ATTACHED");
        DeviceConfiguration deviceConfiguration = new DeviceConfiguration(DeviceConfiguration.ConfigurationType.NOTHING);
        deviceConfiguration.setPort(n);
        this.b(deviceConfiguration);
        this.a(n, 8);
    }

    private void a(LinearLayout linearLayout, String string) {
        TextView textView = (TextView)linearLayout.findViewById(R.id.portNumber);
        int n = Integer.parseInt(textView.getText().toString());
        EditText editText = (EditText)linearLayout.findViewById(R.id.editTextResult_name);
        DeviceConfiguration deviceConfiguration = this.g.get(n);
        editText.setEnabled(true);
        this.a(editText, deviceConfiguration);
        DeviceConfiguration.ConfigurationType configurationType = deviceConfiguration.typeFromString(string);
        if (configurationType == DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER || configurationType == DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER || configurationType == DeviceConfiguration.ConfigurationType.MATRIX_CONTROLLER) {
            this.a(n, string);
            this.a(n, 0);
        } else {
            deviceConfiguration.setType(configurationType);
            if (configurationType == DeviceConfiguration.ConfigurationType.NOTHING) {
                deviceConfiguration.setEnabled(false);
            } else {
                deviceConfiguration.setEnabled(true);
            }
            this.a(n, 8);
        }
        if (a) {
            DeviceConfiguration deviceConfiguration2 = this.g.get(n);
            RobotLog.e((String)("[changeDevice] modules.get(port) name: " + deviceConfiguration2.getName() + ", port: " + deviceConfiguration2.getPort() + ", type: " + (Object)deviceConfiguration2.getType()));
        }
    }

    private void a(int n, String string) {
        DeviceConfiguration deviceConfiguration = this.g.get(n);
        String string2 = deviceConfiguration.getName();
        ArrayList<Object> arrayList = new ArrayList<Object>();
        SerialNumber serialNumber = ControllerConfiguration.NO_SERIAL_NUMBER;
        DeviceConfiguration.ConfigurationType configurationType = deviceConfiguration.getType();
        if (!configurationType.toString().equalsIgnoreCase(string)) {
            ControllerConfiguration controllerConfiguration = new ControllerConfiguration("dummy module", arrayList, serialNumber, DeviceConfiguration.ConfigurationType.NOTHING);
            if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER.toString())) {
                for (int i = 1; i <= 2; ++i) {
                    arrayList.add((Object)new MotorConfiguration(i));
                }
                controllerConfiguration = new MotorControllerConfiguration(string2, arrayList, serialNumber);
                controllerConfiguration.setPort(n);
            } else if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER.toString())) {
                for (int i = 1; i <= 6; ++i) {
                    arrayList.add((Object)new ServoConfiguration(i));
                }
                controllerConfiguration = new ServoControllerConfiguration(string2, arrayList, serialNumber);
                controllerConfiguration.setPort(n);
            } else if (string.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MATRIX_CONTROLLER.toString())) {
                ArrayList<MotorConfiguration> arrayList2 = new ArrayList<MotorConfiguration>();
                for (int i = 1; i <= 4; ++i) {
                    arrayList2.add(new MotorConfiguration(i));
                }
                ArrayList<ServoConfiguration> arrayList3 = new ArrayList<ServoConfiguration>();
                for (int j = 1; j <= 4; ++j) {
                    arrayList3.add(new ServoConfiguration(j));
                }
                controllerConfiguration = new MatrixControllerConfiguration(string2, arrayList2, arrayList3, serialNumber);
                controllerConfiguration.setPort(n);
            }
            controllerConfiguration.setEnabled(true);
            this.b((DeviceConfiguration)controllerConfiguration);
        }
    }

    public void editController_portALL(View v) {
        LinearLayout linearLayout = (LinearLayout)v.getParent().getParent();
        TextView textView = (TextView)linearLayout.findViewById(R.id.portNumber);
        int n = Integer.parseInt(textView.getText().toString());
        DeviceConfiguration deviceConfiguration = this.g.get(n);
        if (!this.c(deviceConfiguration)) {
            Spinner spinner = (Spinner)linearLayout.findViewById(R.id.choiceSpinner_legacyModule);
            String string = spinner.getSelectedItem().toString();
            this.a(n, string);
        }
        this.a(deviceConfiguration);
    }

    private void a(DeviceConfiguration deviceConfiguration) {
        LinearLayout linearLayout = (LinearLayout)this.a(deviceConfiguration.getPort());
        EditText editText = (EditText)linearLayout.findViewById(R.id.editTextResult_name);
        deviceConfiguration.setName(editText.getText().toString());
        if (deviceConfiguration.getType() == DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER) {
            int n = 101;
            Intent intent = new Intent(this.d, (Class)EditMotorControllerActivity.class);
            intent.putExtra("EDIT_MOTOR_CONTROLLER_CONFIG", (Serializable)deviceConfiguration);
            intent.putExtra("requestCode", n);
            this.setResult(-1, intent);
            this.startActivityForResult(intent, n);
        } else if (deviceConfiguration.getType() == DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER) {
            int n = 102;
            Intent intent = new Intent(this.d, (Class)EditServoControllerActivity.class);
            intent.putExtra("Edit Servo ControllerConfiguration Activity", (Serializable)deviceConfiguration);
            this.setResult(-1, intent);
            this.startActivityForResult(intent, n);
        } else if (deviceConfiguration.getType() == DeviceConfiguration.ConfigurationType.MATRIX_CONTROLLER) {
            int n = 103;
            Intent intent = new Intent(this.d, (Class)EditMatrixControllerActivity.class);
            intent.putExtra("Edit Matrix ControllerConfiguration Activity", (Serializable)deviceConfiguration);
            this.setResult(-1, intent);
            this.startActivityForResult(intent, n);
        }
    }

    private void b(DeviceConfiguration deviceConfiguration) {
        int n = deviceConfiguration.getPort();
        this.g.set(n, deviceConfiguration);
    }

    private View a(int n) {
        switch (n) {
            case 0: {
                return this.h;
            }
            case 1: {
                return this.i;
            }
            case 2: {
                return this.j;
            }
            case 3: {
                return this.k;
            }
            case 4: {
                return this.l;
            }
            case 5: {
                return this.m;
            }
        }
        return null;
    }

    private void a(int n, int n2) {
        View view = this.a(n);
        Button button = (Button)view.findViewById(R.id.edit_controller_btn);
        button.setVisibility(n2);
    }

    private boolean c(DeviceConfiguration deviceConfiguration) {
        return deviceConfiguration.getType() == DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER || deviceConfiguration.getType() == DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER;
    }

    private void a(EditText editText, DeviceConfiguration deviceConfiguration) {
        if (editText.getText().toString().equalsIgnoreCase("NO DEVICE ATTACHED")) {
            editText.setText((CharSequence)"");
            deviceConfiguration.setName("");
        } else {
            editText.setText((CharSequence)deviceConfiguration.getName());
        }
    }

    private class a
    implements TextWatcher {
        private int b;
        private boolean c;
        final /* synthetic */ EditLegacyModuleControllerActivity a;

        private a(EditLegacyModuleControllerActivity editLegacyModuleControllerActivity) {
            this.a = editLegacyModuleControllerActivity;
            this.c = false;
            this.c = true;
        }

        private a(EditLegacyModuleControllerActivity editLegacyModuleControllerActivity, View view) {
            this.a = editLegacyModuleControllerActivity;
            this.c = false;
            TextView textView = (TextView)view.findViewById(R.id.portNumber);
            this.b = Integer.parseInt(textView.getText().toString());
        }

        public void beforeTextChanged(CharSequence charSequence, int i, int i1, int i2) {
        }

        public void onTextChanged(CharSequence charSequence, int i, int i1, int i2) {
        }

        public void afterTextChanged(Editable editable) {
            String string = editable.toString();
            if (this.c) {
                this.a.e.setName(string);
            } else {
                DeviceConfiguration deviceConfiguration = (DeviceConfiguration)this.a.g.get(this.b);
                deviceConfiguration.setName(string);
            }
        }
    }

}

