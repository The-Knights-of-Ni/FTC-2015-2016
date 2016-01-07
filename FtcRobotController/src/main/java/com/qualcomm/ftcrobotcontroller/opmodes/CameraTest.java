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
import android.graphics.Canvas;
import android.graphics.Paint;

public class CameraTest extends LinearOpMode
{
    Camera camera = null;
    SurfaceHolder surface_holder = null;
    int camera_w = 0;
    int camera_h = 0;
    Paint paint = null;
    
    byte[] robot_state;
    
    class CameraPreviewCallback implements Camera.PreviewCallback
    {
        CameraPreviewCallback(){}
        public void onPreviewFrame(byte[] data, Camera camera)
        {}
    }
    
    public CameraTest() {
        //camera = FtcRobotControllerActivity.camera;
        surface_holder = FtcRobotControllerActivity.camera_overlay.surface_holder;

        paint = new Paint();
        paint.setARGB(255, 255, 255, 255);
        
        Camera.Size camera_size = camera.getParameters().getPictureSize();
        camera_w = camera_size.width;
        camera_h = camera_size.height;
        
        robot_state = new byte[camera_w*camera_h*4];//camera_testRobotStateElements.robot_state_size];
        camera_testRobotStateElements.robot_state = robot_state;
        
        /* CameraPreviewCallback camera_preview_callback = new CameraPreviewCallback(); */
        
        /* camera.addCallbackBuffer(camera_testRobotStateElements.get_camera_buffer()); */
        /* camera.setPreviewCallbackWithBuffer(camera_preview_callback); */
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
        
        Canvas canvas = surface_holder.lockCanvas();
        canvas.drawARGB(0, 0, 0, 0);
        canvas.drawLine(canvas.getWidth()/2.0f+100.0f*(float)Math.sin(time), 0.0f, canvas.getWidth()/2.0f, canvas.getHeight(), paint);
        surface_holder.unlockCanvasAndPost(canvas);
    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        main();
    }
}
