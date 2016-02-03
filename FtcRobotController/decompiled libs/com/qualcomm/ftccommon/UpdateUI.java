/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Activity
 *  android.content.Context
 *  android.widget.TextView
 *  android.widget.Toast
 *  com.qualcomm.robotcore.hardware.Gamepad
 *  com.qualcomm.robotcore.util.Dimmer
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.Util
 *  com.qualcomm.robotcore.wifi.WifiDirectAssistant
 *  com.qualcomm.robotcore.wifi.WifiDirectAssistant$Event
 */
package com.qualcomm.ftccommon;

import android.app.Activity;
import android.content.Context;
import android.widget.TextView;
import android.widget.Toast;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Util;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;

public class UpdateUI {
    protected TextView textDeviceName;
    protected TextView textWifiDirectStatus;
    protected TextView textRobotStatus;
    protected TextView[] textGamepad = new TextView[2];
    protected TextView textOpMode;
    protected TextView textErrorMessage;
    Restarter a;
    FtcRobotControllerService b;
    Activity c;
    Dimmer d;

    public UpdateUI(Activity activity, Dimmer dimmer) {
        this.c = activity;
        this.d = dimmer;
    }

    public void setTextViews(TextView textWifiDirectStatus, TextView textRobotStatus, TextView[] textGamepad, TextView textOpMode, TextView textErrorMessage, TextView textDeviceName) {
        this.textWifiDirectStatus = textWifiDirectStatus;
        this.textRobotStatus = textRobotStatus;
        this.textGamepad = textGamepad;
        this.textOpMode = textOpMode;
        this.textErrorMessage = textErrorMessage;
        this.textDeviceName = textDeviceName;
    }

    public void setControllerService(FtcRobotControllerService controllerService) {
        this.b = controllerService;
    }

    public void setRestarter(Restarter restarter) {
        this.a = restarter;
    }

    private void a(String string) {
        DbgLog.msg(string);
        final String string2 = string;
        this.c.runOnUiThread(new Runnable(){

            @Override
            public void run() {
                UpdateUI.this.textWifiDirectStatus.setText((CharSequence)string2);
            }
        });
    }

    private void b(final String string) {
        this.c.runOnUiThread(new Runnable(){

            @Override
            public void run() {
                UpdateUI.this.textDeviceName.setText((CharSequence)string);
            }
        });
    }

    private void a() {
        this.a.requestRestart();
    }

    public class Callback {
        public void restartRobot() {
            UpdateUI.this.c.runOnUiThread(new Runnable(){

                @Override
                public void run() {
                    Toast.makeText((Context)UpdateUI.this.c, (CharSequence)"Restarting Robot", (int)0).show();
                }
            });
            Thread thread = new Thread(new Runnable(){

                @Override
                public void run() {
                    Util.logThreadLifeCycle((String)"restart robot UI thunker", (Runnable)new Runnable(){

                        @Override
                        public void run() {
                            try {
                                Thread.sleep(1500);
                            }
                            catch (InterruptedException var1_1) {
                                // empty catch block
                            }
                            UpdateUI.this.c.runOnUiThread(new Runnable(){

                                @Override
                                public void run() {
                                    UpdateUI.this.a();
                                }
                            });
                        }

                    });
                }

            });
            thread.start();
        }

        public void updateUi(final String opModeName, final Gamepad[] gamepads) {
            UpdateUI.this.c.runOnUiThread(new Runnable(){

                @Override
                public void run() {
                    for (int i = 0; i < UpdateUI.this.textGamepad.length && i < gamepads.length; ++i) {
                        if (gamepads[i].id == -1) {
                            UpdateUI.this.textGamepad[i].setText((CharSequence)" ");
                            continue;
                        }
                        UpdateUI.this.textGamepad[i].setText((CharSequence)gamepads[i].toString());
                    }
                    UpdateUI.this.textOpMode.setText((CharSequence)("Op Mode: " + opModeName));
                    UpdateUI.this.textErrorMessage.setText((CharSequence)RobotLog.getGlobalErrorMsg());
                }
            });
        }

        public void wifiDirectUpdate(WifiDirectAssistant.Event event) {
            String string = "Wifi Direct - ";
            switch (event) {
                case DISCONNECTED: {
                    UpdateUI.this.a("Wifi Direct - disconnected");
                    break;
                }
                case CONNECTED_AS_GROUP_OWNER: {
                    UpdateUI.this.a("Wifi Direct - enabled");
                    break;
                }
                case ERROR: {
                    UpdateUI.this.a("Wifi Direct - ERROR");
                    break;
                }
                case CONNECTION_INFO_AVAILABLE: {
                    WifiDirectAssistant wifiDirectAssistant = UpdateUI.this.b.getWifiDirectAssistant();
                    UpdateUI.this.b(wifiDirectAssistant.getDeviceName());
                }
            }
        }

        public void robotUpdate(final String status) {
            DbgLog.msg(status);
            UpdateUI.this.c.runOnUiThread(new Runnable(){

                @Override
                public void run() {
                    UpdateUI.this.textRobotStatus.setText((CharSequence)status);
                    UpdateUI.this.textErrorMessage.setText((CharSequence)RobotLog.getGlobalErrorMsg());
                    if (RobotLog.hasGlobalErrorMsg()) {
                        UpdateUI.this.d.longBright();
                    }
                }
            });
        }

    }

}

