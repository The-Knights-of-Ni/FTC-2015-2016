/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Activity
 *  android.app.ProgressDialog
 *  android.content.Context
 *  android.content.Intent
 *  android.net.wifi.WifiManager
 *  android.os.Bundle
 *  android.view.View
 *  android.widget.TextView
 *  com.qualcomm.ftccommon.R
 *  com.qualcomm.ftccommon.R$id
 *  com.qualcomm.ftccommon.R$layout
 *  com.qualcomm.ftccommon.R$style
 *  com.qualcomm.robotcore.wifi.FixWifiDirectSetup
 */
package com.qualcomm.ftccommon;

import android.app.Activity;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.Intent;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.R;
import com.qualcomm.robotcore.wifi.FixWifiDirectSetup;

public class ConfigWifiDirectActivity
extends Activity {
    private static Flag a = Flag.NONE;
    private WifiManager b;
    private ProgressDialog c;
    private Context d;
    private TextView e;
    private TextView f;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.setContentView(R.layout.activity_config_wifi_direct);
        this.e = (TextView)this.findViewById(R.id.textPleaseWait);
        this.f = (TextView)this.findViewById(R.id.textBadDeviceName);
        this.d = this;
    }

    protected void onResume() {
        super.onResume();
        this.e.setVisibility(0);
        this.b = (WifiManager)this.getSystemService("wifi");
        DbgLog.msg("Processing flag " + a.toString());
        switch (a) {
            case WIFI_DIRECT_DEVICE_NAME_INVALID: {
                new Thread(new a()).start();
                break;
            }
            case WIFI_DIRECT_FIX_CONFIG: {
                new Thread(new b()).start();
            }
        }
    }

    protected void onPause() {
        super.onPause();
        a = Flag.NONE;
        this.f.setVisibility(4);
    }

    private void a() {
        this.runOnUiThread(new Runnable(){

            @Override
            public void run() {
                ConfigWifiDirectActivity.this.c = new ProgressDialog(ConfigWifiDirectActivity.this.d, R.style.CustomAlertDialog);
                ConfigWifiDirectActivity.this.c.setMessage((CharSequence)"Please wait");
                ConfigWifiDirectActivity.this.c.setTitle((CharSequence)"Configuring Wifi Direct");
                ConfigWifiDirectActivity.this.c.setIndeterminate(true);
                ConfigWifiDirectActivity.this.c.show();
            }
        });
    }

    private void b() {
        this.runOnUiThread(new Runnable(){

            @Override
            public void run() {
                ConfigWifiDirectActivity.this.c.dismiss();
            }
        });
    }

    public static void launch(Context context) {
        ConfigWifiDirectActivity.launch(context, Flag.WIFI_DIRECT_FIX_CONFIG);
    }

    public static void launch(Context context, Flag flag) {
        Intent intent = new Intent(context, (Class)ConfigWifiDirectActivity.class);
        intent.addFlags(1342177280);
        context.startActivity(intent);
        a = flag;
    }

    private class a
    implements Runnable {
        private a() {
        }

        @Override
        public void run() {
            DbgLog.msg("attempting to disable Wifi due to bad wifi direct device name");
            ConfigWifiDirectActivity.this.a();
            try {
                FixWifiDirectSetup.disableWifiDirect((WifiManager)ConfigWifiDirectActivity.this.b);
            }
            catch (InterruptedException var1_1) {
                DbgLog.error("Cannot fix wifi setup - interrupted");
            }
            ConfigWifiDirectActivity.this.runOnUiThread(new Runnable(){

                @Override
                public void run() {
                    ConfigWifiDirectActivity.this.e.setVisibility(4);
                    ConfigWifiDirectActivity.this.f.setVisibility(0);
                }
            });
            ConfigWifiDirectActivity.this.b();
        }

    }

    private class b
    implements Runnable {
        private b() {
        }

        @Override
        public void run() {
            DbgLog.msg("attempting to reconfigure Wifi Direct");
            ConfigWifiDirectActivity.this.a();
            try {
                FixWifiDirectSetup.fixWifiDirectSetup((WifiManager)ConfigWifiDirectActivity.this.b);
            }
            catch (InterruptedException var1_1) {
                DbgLog.error("Cannot fix wifi setup - interrupted");
            }
            ConfigWifiDirectActivity.this.b();
            ConfigWifiDirectActivity.this.runOnUiThread(new Runnable(){

                @Override
                public void run() {
                    ConfigWifiDirectActivity.this.finish();
                }
            });
        }

    }

    public static enum Flag {
        NONE,
        WIFI_DIRECT_FIX_CONFIG,
        WIFI_DIRECT_DEVICE_NAME_INVALID;
        

        private Flag() {
        }
    }

}

