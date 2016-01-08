package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

import com.qualcomm.ftcrobotcontroller.opmodes.camera_testRobotStateElements;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.DbgLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import android.hardware.Camera;
import android.view.SurfaceHolder;
import android.view.Surface;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff.Mode;
import android.graphics.ImageFormat;

public class CameraTest extends LinearOpMode
{
    Camera camera = null;
    SurfaceHolder surface_holder = null;
    Surface surface = null;
    int camera_w = 0;
    int camera_h = 0;
    Paint paint = null;
    byte[] robot_state;
    CameraPreviewCallback camera_preview_callback;

    byte[] camera_buffer = null;
    
    class CameraPreviewCallback implements Camera.PreviewCallback
    {
        
        
        CameraPreviewCallback(){}
        public void onPreviewFrame(byte[] data, Camera camera)
        {
            camera.addCallbackBuffer(camera_buffer);
        }
    }

    //TODO: look into latency, it might just be a delay when it draws to the canvas
    public CameraTest()
    {
        camera = FtcRobotControllerActivity.camera;
        surface_holder = FtcRobotControllerActivity.camera_overlay.surface_holder;
        surface = surface_holder.getSurface();
        
        paint = new Paint();
        paint.setARGB(255, 255, 255, 255);
        
        Camera.Parameters parameters = camera.getParameters();
        Camera.Size camera_size = parameters.getPreviewSize();
        camera_w = camera_size.width;
        camera_h = camera_size.height;
        parameters.setPreviewFormat(ImageFormat.NV21);
        parameters.setExposureCompensation(0);
        parameters.setWhiteBalance(Camera.Parameters.WHITE_BALANCE_INCANDESCENT);
        parameters.set("iso", "ISO100");
        parameters.set("max-exposure-time", 2000000);
        parameters.set("min-exposure-time", 2000000);
        DbgLog.error("Camera parameters: "+parameters.flatten());
        /* int[][] fps_ranges = parameters.getSupportedPreviewFpsRange().toArray(new int[20][2]); */
        /* for(int i = 0; i < fps_ranges.length; i++) */
        /* { */
        /*     DbgLog.error("fps range" + String.format("%d, %d", fps_ranges[i][0], fps_ranges[i][1])); */
        /* } */
        camera.setParameters(parameters);
        
        robot_state = new byte[camera_testRobotStateElements.robot_state_size];
        camera_testRobotStateElements.robot_state = robot_state;
        camera_testRobotStateElements.set_camera_w(camera_w);
        camera_testRobotStateElements.set_camera_h(camera_h);
        
        camera_buffer = new byte[camera_w*camera_h*4];
        camera_preview_callback = new CameraPreviewCallback();
        
        camera.setPreviewCallbackWithBuffer(camera_preview_callback);
        camera.addCallbackBuffer(camera_buffer);
        //camera.startPreview();
    }
    
    native void main();
    
    static
    {
        System.loadLibrary("camera_test");
    }
    
    void applyRobotState()
    {
        telemetry.addData("width", String.format("%d", camera_w));
        telemetry.addData("height", String.format("%d", camera_h));
        telemetry.addData("data_length", String.format("%d", camera_buffer.length));
        
        /* Canvas canvas = surface_holder.lockCanvas(); */
        /* canvas.drawColor(Color.TRANSPARENT, Mode.CLEAR); */
        /* canvas.drawLine(canvas.getWidth()/2.0f+100.0f*(float)Math.sin(time), 0.0f, canvas.getWidth()/2.0f, canvas.getHeight(), paint); */
        /* surface_holder.unlockCanvasAndPost(canvas); */
    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        main();
    }
}
