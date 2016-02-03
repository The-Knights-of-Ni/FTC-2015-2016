/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Activity
 *  android.content.Context
 *  android.content.pm.PackageInfo
 *  android.content.pm.PackageManager
 *  android.content.pm.PackageManager$NameNotFoundException
 *  android.view.View
 *  android.view.ViewGroup
 *  android.widget.ArrayAdapter
 *  android.widget.ListAdapter
 *  android.widget.ListView
 *  android.widget.TextView
 *  com.qualcomm.ftccommon.R
 *  com.qualcomm.ftccommon.R$id
 *  com.qualcomm.ftccommon.R$layout
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.Version
 *  com.qualcomm.robotcore.wifi.WifiDirectAssistant
 */
package com.qualcomm.ftccommon;

import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.TextView;
import com.qualcomm.ftccommon.R;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Version;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;
import java.net.InetAddress;

public class AboutActivity
extends Activity {
    WifiDirectAssistant a = null;

    protected void onStart() {
        super.onStart();
        this.setContentView(R.layout.activity_about);
        ListView listView = (ListView)this.findViewById(R.id.aboutList);
        try {
            this.a = WifiDirectAssistant.getWifiDirectAssistant((Context)null);
            this.a.enable();
        }
        catch (NullPointerException var2_2) {
            RobotLog.i((String)"Cannot start Wifi Direct Assistant");
            this.a = null;
        }
        ArrayAdapter<Item> arrayAdapter = new ArrayAdapter<Item>((Context)this, 17367044, 16908308){

            public View getView(int position, View convertView, ViewGroup parent) {
                View view = super.getView(position, convertView, parent);
                TextView textView = (TextView)view.findViewById(16908308);
                TextView textView2 = (TextView)view.findViewById(16908309);
                Item item = this.a(position);
                textView.setText((CharSequence)item.title);
                textView2.setText((CharSequence)item.info);
                return view;
            }

            public int getCount() {
                return 3;
            }

            public Item a(int n) {
                switch (n) {
                    case 0: {
                        return this.a();
                    }
                    case 1: {
                        return this.b();
                    }
                    case 2: {
                        return this.c();
                    }
                }
                return new Item();
            }

            private Item a() {
                Item item = new Item();
                item.title = "App Version";
                try {
                    item.info = AboutActivity.this.getPackageManager().getPackageInfo((String)AboutActivity.this.getPackageName(), (int)0).versionName;
                }
                catch (PackageManager.NameNotFoundException var2_2) {
                    item.info = var2_2.getMessage();
                }
                return item;
            }

            private Item b() {
                Item item = new Item();
                item.title = "Library Version";
                item.info = Version.getLibraryVersion();
                return item;
            }

            private Item c() {
                Item item = new Item();
                item.title = "Wifi Direct Information";
                item.info = "unavailable";
                StringBuilder stringBuilder = new StringBuilder();
                if (AboutActivity.this.a != null && AboutActivity.this.a.isEnabled()) {
                    stringBuilder.append("Name: ").append(AboutActivity.this.a.getDeviceName());
                    if (AboutActivity.this.a.isGroupOwner()) {
                        stringBuilder.append("\nIP Address").append(AboutActivity.this.a.getGroupOwnerAddress().getHostAddress());
                        stringBuilder.append("\nPassphrase: ").append(AboutActivity.this.a.getPassphrase());
                        stringBuilder.append("\nGroup Owner");
                    } else if (AboutActivity.this.a.isConnected()) {
                        stringBuilder.append("\nGroup Owner: ").append(AboutActivity.this.a.getGroupOwnerName());
                        stringBuilder.append("\nConnected");
                    } else {
                        stringBuilder.append("\nNo connection information");
                    }
                    item.info = stringBuilder.toString();
                }
                return item;
            }

            public /* synthetic */ Object getItem(int n) {
                return this.a(n);
            }
        };
        listView.setAdapter((ListAdapter)arrayAdapter);
    }

    protected void onStop() {
        super.onStop();
        if (this.a != null) {
            this.a.disable();
        }
    }

    public static class Item {
        public String title = "";
        public String info = "";
    }

}

