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

public class CameraTest extends LinearOpMode
{
    Camera camera = null;
    SurfaceHolder surface_holder = null;
    Surface surface = null;
    int camera_w = 0;
    int camera_h = 0;
    Paint paint = null;
    byte[] robot_state;
    boolean prev_camera_waiting = false;
    CameraPreviewCallback camera_preview_callback;

    byte[] camera_data = new byte[10];
    
    class CameraPreviewCallback implements Camera.PreviewCallback
    {
        volatile boolean camera_waiting;
        CameraPreviewCallback(){}
        public void onPreviewFrame(byte[] data, Camera camera)
        {
            camera_waiting = false;
            
            //TODO: try to get rid of this array copy
            System.arraycopy(data, 0,
                             camera_testRobotStateElements.get_current_buffer(), 76800*(1-camera_testRobotStateElements.get_current_buffer()),
                             4915200);
            
            
            camera.addCallbackBuffer(ByteBuffer.wrap(robot_state, 4915200*camera_testRobotStateElements.get_current_buffer(), 76800).order(ByteOrder.nativeOrder()).array());
            camera_testRobotStateElements.set_current_buffer((camera_testRobotStateElements.get_current_buffer() == 1) ? 0:1);
        }
    }
    
    public CameraTest()
    {
        camera = FtcRobotControllerActivity.camera;
        surface_holder = FtcRobotControllerActivity.camera_overlay.surface_holder;
        surface = surface_holder.getSurface();
        
        paint = new Paint();
        paint.setARGB(255, 255, 255, 255);
        
        Camera.Size camera_size = camera.getParameters().getPreviewSize();
        camera_w = camera_size.width;
        camera_h = camera_size.height;
        
        robot_state = new byte[camera_testRobotStateElements.robot_state_size];
        camera_testRobotStateElements.robot_state = robot_state;
        
        camera_preview_callback = new CameraPreviewCallback();
        
        camera.setPreviewCallback/* WithBuffer */(camera_preview_callback);
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
        telemetry.addData("data_length", String.format("%d", camera_data.length));
        
        if(true || camera_preview_callback.camera_waiting && !prev_camera_waiting)
        {
            camera_preview_callback.camera_waiting = true;
        }
        prev_camera_waiting = camera_preview_callback.camera_waiting;
        
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
