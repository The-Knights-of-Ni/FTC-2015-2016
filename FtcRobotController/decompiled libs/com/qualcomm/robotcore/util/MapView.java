/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  android.content.res.Resources
 *  android.graphics.Bitmap
 *  android.graphics.Bitmap$Config
 *  android.graphics.BitmapFactory
 *  android.graphics.Canvas
 *  android.graphics.Matrix
 *  android.graphics.Paint
 *  android.graphics.drawable.BitmapDrawable
 *  android.graphics.drawable.Drawable
 *  android.util.AttributeSet
 *  android.util.Log
 *  android.view.View
 */
package com.qualcomm.robotcore.util;

import android.content.Context;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import java.util.Collection;
import java.util.HashMap;

public class MapView
extends View {
    private int b;
    private int c;
    private int d;
    private int e;
    private Paint f;
    private Canvas g;
    private Bitmap h;
    private boolean i = false;
    private boolean j = false;
    MapView a;
    private int k = 1;
    private float l;
    private float m;
    private BitmapDrawable n;
    private int o;
    private int p;
    private int q;
    private boolean r = false;
    private HashMap<Integer, a> s;
    private Bitmap t;

    protected void onSizeChanged(int x, int y, int oldx, int oldy) {
        this.l = (float)this.getWidth() / (float)this.b;
        this.m = (float)this.getHeight() / (float)this.c;
        this.j = true;
        this.redraw();
        Log.e((String)"MapView", (String)"Size changed");
    }

    public MapView(Context context) {
        super(context);
        this.a();
    }

    public MapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.a();
    }

    public MapView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        this.a();
    }

    private void a() {
        this.f = new Paint();
        this.f.setColor(-16777216);
        this.f.setStrokeWidth(1.0f);
        this.f.setAntiAlias(true);
        this.a = this;
        this.s = new HashMap();
    }

    private int a(int n) {
        if (n % 2 == 0) {
            return n;
        }
        return n + 1;
    }

    public void setup(int xMax, int yMax, int numLinesX, int numLinesY, Bitmap robotIcon) {
        this.b = xMax * 2;
        this.c = yMax * 2;
        this.d = this.b / this.a(numLinesX);
        this.e = this.c / this.a(numLinesY);
        this.t = robotIcon;
        this.i = true;
    }

    private void b() {
        float f;
        int n;
        this.h = Bitmap.createBitmap((int)this.getWidth(), (int)this.getHeight(), (Bitmap.Config)Bitmap.Config.ARGB_8888);
        this.g = new Canvas(this.h);
        Paint paint = new Paint();
        paint.setColor(-1);
        paint.setAntiAlias(true);
        this.g.drawRect(0.0f, 0.0f, (float)this.g.getWidth(), (float)this.g.getHeight(), paint);
        for (n = 0; n < this.c; n+=this.e) {
            f = (float)n * this.m;
            this.g.drawLine(0.0f, f, (float)this.g.getWidth(), f, this.f);
        }
        for (n = 0; n < this.b; n+=this.d) {
            f = (float)n * this.l;
            this.g.drawLine(f, 0.0f, f, (float)this.g.getHeight(), this.f);
        }
    }

    private float b(int n) {
        float f = (float)n * this.l;
        return f+=(float)(this.getWidth() / 2);
    }

    private float c(int n) {
        float f = (float)n * this.m;
        f = (float)(this.getHeight() / 2) - f;
        return f;
    }

    private int d(int n) {
        return 360 - n;
    }

    public void setRobotLocation(int x, int y, int angle) {
        this.o = - x;
        this.p = y;
        this.q = angle;
        this.r = true;
    }

    public int addMarker(int x, int y, int color) {
        int n = this.k++;
        a a = new a(n, - x, y, color, true);
        this.s.put(n, a);
        return n;
    }

    public boolean removeMarker(int id) {
        a a = this.s.remove(id);
        if (a == null) {
            return false;
        }
        return true;
    }

    public int addDrawable(int x, int y, int resource) {
        int n = this.k++;
        a a = new a(n, - x, y, resource, false);
        this.s.put(n, a);
        return n;
    }

    private void c() {
        for (a a : this.s.values()) {
            Paint paint;
            float f = this.b(a.b);
            float f2 = this.c(a.c);
            if (a.e) {
                paint = new Paint();
                paint.setColor(a.d);
                this.g.drawCircle(f, f2, 5.0f, paint);
                continue;
            }
            paint = BitmapFactory.decodeResource((Resources)this.getResources(), (int)a.d);
            this.g.drawBitmap((Bitmap)paint, f-=(float)(paint.getWidth() / 2), f2-=(float)(paint.getHeight() / 2), new Paint());
        }
    }

    private void d() {
        float f = this.b(this.o);
        float f2 = this.c(this.p);
        int n = this.d(this.q);
        Matrix matrix = new Matrix();
        matrix.postRotate((float)n);
        matrix.postScale(0.2f, 0.2f);
        Bitmap bitmap = this.t;
        Bitmap bitmap2 = Bitmap.createBitmap((Bitmap)bitmap, (int)0, (int)0, (int)bitmap.getWidth(), (int)bitmap.getHeight(), (Matrix)matrix, (boolean)true);
        this.g.drawBitmap(bitmap2, f-=(float)(bitmap2.getWidth() / 2), f2-=(float)(bitmap2.getHeight() / 2), new Paint());
    }

    public void redraw() {
        if (this.i && this.j) {
            this.b();
            this.c();
            if (this.r) {
                this.d();
            }
        }
        this.n = new BitmapDrawable(this.getResources(), this.h);
        this.a.setBackgroundDrawable((Drawable)this.n);
    }

    private class a {
        public int a;
        public int b;
        public int c;
        public int d;
        public boolean e;

        public a(int n, int n2, int n3, int n4, boolean bl) {
            this.a = n;
            this.b = n2;
            this.c = n3;
            this.d = n4;
            this.e = bl;
        }
    }

}

