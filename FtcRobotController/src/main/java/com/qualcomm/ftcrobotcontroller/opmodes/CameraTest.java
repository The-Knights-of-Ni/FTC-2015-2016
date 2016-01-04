package com.qualcomm.ftcrobotcontroller.opmodes;

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

public class CameraTest extends LinearOpMode
{
    Camera camera = null;
    
    byte[] robot_state;
    
    public CameraTest() {
        robot_state = new byte[camera_testRobotStateElements.robot_state_size];
        camera_testRobotStateElements.robot_state = robot_state;

        try
        {
            camera = Camera.open();
        }
        catch(Exception e)
        {
            DbgLog.error("could not open camera, camera is in use or does not exist");
        }
        
        camera.addCallbackBuffer(camera_testRobotStateElements.get_camera_buffer());
        camera.setPreviewCallbackWithBuffer();
    }
    
    native void main();
    
    static
    {
        System.loadLibrary("test");
    }
    
    
    void applyRobotState()
    {
        
    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        main();
    }

    //TODO: add preview stuff to FTCRobotControllerActivity and the layout xml
    public class CameraPreview extends SurfaceView implements SurfaceHolder.Callback
    {
        public SurfaceHolder surface_holder;
        public Camera camera;
        
        public CameraPreview(Context context, Camera cam)
        {
            super(context);
            camera = cam;
            surface_holder = getHolder();
            surface_holder.addCallback(this);
        }
        
        public void surfaceCreated(SurfaceHolder holder)
        {
            try
            {
                camera,setPreviewDisplay(holder);
                camer.startPreview();
            }
            catch(IOException e)
            {
                DbgLog.error("error setting camera preview: " e.getMessage());
            }
        }
        
        public void sufaceDestroyed(SurfaceHolder holder){}
        
        public void surfaceChanged(SurfaceHolder holder, format, int w, int h)
        {
            if(surface_holder.getSurface() == null)
            {
                return;
            }
            
            try
            {
                camera.stopPreview();
            }
            catch(Exception e)
            {
                //Do nothing
            }
            
            try
            {
                camera.setPreviewDisplay(surface_holder);
                camera.startPreview();
            }
            catch(Exception e)
            {
                DbgLog.error("error starting camera preview: ", e.getMessage());
            }
        }
    }
}
