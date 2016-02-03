/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.AccelerationSensor
 *  com.qualcomm.robotcore.hardware.AnalogInput
 *  com.qualcomm.robotcore.hardware.AnalogInputController
 *  com.qualcomm.robotcore.hardware.AnalogOutput
 *  com.qualcomm.robotcore.hardware.AnalogOutputController
 *  com.qualcomm.robotcore.hardware.ColorSensor
 *  com.qualcomm.robotcore.hardware.CompassSensor
 *  com.qualcomm.robotcore.hardware.DcMotor
 *  com.qualcomm.robotcore.hardware.DcMotorController
 *  com.qualcomm.robotcore.hardware.DeviceInterfaceModule
 *  com.qualcomm.robotcore.hardware.DeviceManager
 *  com.qualcomm.robotcore.hardware.DigitalChannel
 *  com.qualcomm.robotcore.hardware.DigitalChannelController
 *  com.qualcomm.robotcore.hardware.GyroSensor
 *  com.qualcomm.robotcore.hardware.HardwareMap
 *  com.qualcomm.robotcore.hardware.HardwareMap$DeviceMapping
 *  com.qualcomm.robotcore.hardware.I2cController
 *  com.qualcomm.robotcore.hardware.I2cDevice
 *  com.qualcomm.robotcore.hardware.IrSeekerSensor
 *  com.qualcomm.robotcore.hardware.LED
 *  com.qualcomm.robotcore.hardware.LegacyModule
 *  com.qualcomm.robotcore.hardware.LightSensor
 *  com.qualcomm.robotcore.hardware.OpticalDistanceSensor
 *  com.qualcomm.robotcore.hardware.PWMOutput
 *  com.qualcomm.robotcore.hardware.Servo
 *  com.qualcomm.robotcore.hardware.ServoController
 *  com.qualcomm.robotcore.hardware.TouchSensor
 *  com.qualcomm.robotcore.hardware.TouchSensorMultiplexer
 *  com.qualcomm.robotcore.hardware.UltrasonicSensor
 *  com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration$ConfigurationType
 *  com.qualcomm.robotcore.hardware.configuration.DeviceInterfaceModuleConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MatrixControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration
 *  com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler
 *  com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.hardware;

import android.content.Context;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.matrix.MatrixDcMotorController;
import com.qualcomm.hardware.matrix.MatrixMasterController;
import com.qualcomm.hardware.matrix.MatrixServoController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.AnalogOutputController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceInterfaceModuleConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MatrixControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import java.io.InputStream;
import java.util.List;

public class HardwareFactory {
    private Context a;
    private InputStream b = null;

    public HardwareFactory(Context context) {
        this.a = context;
    }

    public HardwareMap createHardwareMap(EventLoopManager manager) throws RobotCoreException, InterruptedException {
        if (this.b == null) {
            throw new RobotCoreException("XML input stream is null, HardwareFactory cannot create a hardware map");
        }
        HardwareMap hardwareMap = new HardwareMap();
        RobotLog.v((String)"Starting Modern Robotics device manager");
        HardwareDeviceManager hardwareDeviceManager = new HardwareDeviceManager(this.a, manager);
        ReadXMLFileHandler readXMLFileHandler = new ReadXMLFileHandler(this.a);
        List list = readXMLFileHandler.parse(this.b);
        block6 : for (ControllerConfiguration controllerConfiguration : list) {
            DeviceConfiguration.ConfigurationType configurationType = controllerConfiguration.getType();
            switch (configurationType) {
                case MOTOR_CONTROLLER: {
                    this.a(hardwareMap, hardwareDeviceManager, controllerConfiguration);
                    continue block6;
                }
                case SERVO_CONTROLLER: {
                    this.b(hardwareMap, hardwareDeviceManager, controllerConfiguration);
                    continue block6;
                }
                case LEGACY_MODULE_CONTROLLER: {
                    this.d(hardwareMap, hardwareDeviceManager, controllerConfiguration);
                    continue block6;
                }
                case DEVICE_INTERFACE_MODULE: {
                    this.c(hardwareMap, hardwareDeviceManager, controllerConfiguration);
                    continue block6;
                }
            }
            RobotLog.w((String)("Unexpected controller type while parsing XML: " + configurationType.toString()));
        }
        hardwareMap.appContext = this.a;
        return hardwareMap;
    }

    public void setXmlInputStream(InputStream xmlInputStream) {
        this.b = xmlInputStream;
    }

    public InputStream getXmlInputStream() {
        return this.b;
    }

    public static void enableDeviceEmulation() {
        HardwareDeviceManager.enableDeviceEmulation();
    }

    public static void disableDeviceEmulation() {
        HardwareDeviceManager.disableDeviceEmulation();
    }

    private void a(HardwareMap hardwareMap, DeviceManager deviceManager, ControllerConfiguration controllerConfiguration) throws RobotCoreException, InterruptedException {
        ModernRoboticsUsbDcMotorController modernRoboticsUsbDcMotorController = (ModernRoboticsUsbDcMotorController)deviceManager.createUsbDcMotorController(controllerConfiguration.getSerialNumber());
        hardwareMap.dcMotorController.put(controllerConfiguration.getName(), (Object)modernRoboticsUsbDcMotorController);
        for (DeviceConfiguration deviceConfiguration : controllerConfiguration.getDevices()) {
            if (!deviceConfiguration.isEnabled()) continue;
            DcMotor dcMotor = deviceManager.createDcMotor((DcMotorController)modernRoboticsUsbDcMotorController, deviceConfiguration.getPort());
            hardwareMap.dcMotor.put(deviceConfiguration.getName(), (Object)dcMotor);
        }
        Object object = modernRoboticsUsbDcMotorController;
        hardwareMap.voltageSensor.put(controllerConfiguration.getName(), object);
    }

    private void b(HardwareMap hardwareMap, DeviceManager deviceManager, ControllerConfiguration controllerConfiguration) throws RobotCoreException, InterruptedException {
        ServoController servoController = deviceManager.createUsbServoController(controllerConfiguration.getSerialNumber());
        hardwareMap.servoController.put(controllerConfiguration.getName(), (Object)servoController);
        for (DeviceConfiguration deviceConfiguration : controllerConfiguration.getDevices()) {
            if (!deviceConfiguration.isEnabled()) continue;
            Servo servo = deviceManager.createServo(servoController, deviceConfiguration.getPort());
            hardwareMap.servo.put(deviceConfiguration.getName(), (Object)servo);
        }
    }

    private void c(HardwareMap hardwareMap, DeviceManager deviceManager, ControllerConfiguration controllerConfiguration) throws RobotCoreException, InterruptedException {
        DeviceInterfaceModule deviceInterfaceModule = deviceManager.createDeviceInterfaceModule(controllerConfiguration.getSerialNumber());
        hardwareMap.deviceInterfaceModule.put(controllerConfiguration.getName(), (Object)deviceInterfaceModule);
        List list = ((DeviceInterfaceModuleConfiguration)controllerConfiguration).getPwmDevices();
        this.a(list, hardwareMap, deviceManager, deviceInterfaceModule);
        List list2 = ((DeviceInterfaceModuleConfiguration)controllerConfiguration).getI2cDevices();
        this.a(list2, hardwareMap, deviceManager, deviceInterfaceModule);
        List list3 = ((DeviceInterfaceModuleConfiguration)controllerConfiguration).getAnalogInputDevices();
        this.a(list3, hardwareMap, deviceManager, deviceInterfaceModule);
        List list4 = ((DeviceInterfaceModuleConfiguration)controllerConfiguration).getDigitalDevices();
        this.a(list4, hardwareMap, deviceManager, deviceInterfaceModule);
        List list5 = ((DeviceInterfaceModuleConfiguration)controllerConfiguration).getAnalogOutputDevices();
        this.a(list5, hardwareMap, deviceManager, deviceInterfaceModule);
    }

    private void a(List<DeviceConfiguration> list, HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule) {
        block15 : for (DeviceConfiguration deviceConfiguration : list) {
            if (!deviceConfiguration.isEnabled()) continue;
            DeviceConfiguration.ConfigurationType configurationType = deviceConfiguration.getType();
            switch (configurationType) {
                case OPTICAL_DISTANCE_SENSOR: {
                    this.h(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case ANALOG_INPUT: {
                    this.d(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case TOUCH_SENSOR: {
                    this.c(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case DIGITAL_DEVICE: {
                    this.b(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case PULSE_WIDTH_DEVICE: {
                    this.e(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case IR_SEEKER_V3: {
                    this.a(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case I2C_DEVICE: {
                    this.f(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case ANALOG_OUTPUT: {
                    this.g(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case ADAFRUIT_COLOR_SENSOR: {
                    this.i(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case LED: {
                    this.a(hardwareMap, deviceManager, (DigitalChannelController)deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case COLOR_SENSOR: {
                    this.j(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case GYRO: {
                    this.k(hardwareMap, deviceManager, deviceInterfaceModule, deviceConfiguration);
                    continue block15;
                }
                case NOTHING: {
                    continue block15;
                }
            }
            RobotLog.w((String)("Unexpected device type connected to Device Interface Module while parsing XML: " + configurationType.toString()));
        }
    }

    private void d(HardwareMap hardwareMap, DeviceManager deviceManager, ControllerConfiguration controllerConfiguration) throws RobotCoreException, InterruptedException {
        LegacyModule legacyModule = deviceManager.createUsbLegacyModule(controllerConfiguration.getSerialNumber());
        hardwareMap.legacyModule.put(controllerConfiguration.getName(), (Object)legacyModule);
        block15 : for (DeviceConfiguration deviceConfiguration : controllerConfiguration.getDevices()) {
            if (!deviceConfiguration.isEnabled()) continue;
            DeviceConfiguration.ConfigurationType configurationType = deviceConfiguration.getType();
            switch (configurationType) {
                case GYRO: {
                    this.e(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case COMPASS: {
                    this.f(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case IR_SEEKER: {
                    this.g(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case LIGHT_SENSOR: {
                    this.h(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case ACCELEROMETER: {
                    this.i(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case MOTOR_CONTROLLER: {
                    this.j(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case SERVO_CONTROLLER: {
                    this.k(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case TOUCH_SENSOR: {
                    this.a(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case TOUCH_SENSOR_MULTIPLEXER: {
                    this.b(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case ULTRASONIC_SENSOR: {
                    this.c(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case COLOR_SENSOR: {
                    this.d(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case MATRIX_CONTROLLER: {
                    this.l(hardwareMap, deviceManager, legacyModule, deviceConfiguration);
                    continue block15;
                }
                case NOTHING: {
                    continue block15;
                }
            }
            RobotLog.w((String)("Unexpected device type connected to Legacy Module while parsing XML: " + configurationType.toString()));
        }
    }

    private void a(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        IrSeekerSensor irSeekerSensor = deviceManager.createI2cIrSeekerSensorV3(deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.irSeekerSensor.put(deviceConfiguration.getName(), (Object)irSeekerSensor);
    }

    private void b(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        DigitalChannel digitalChannel = deviceManager.createDigitalChannelDevice((DigitalChannelController)deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.digitalChannel.put(deviceConfiguration.getName(), (Object)digitalChannel);
    }

    private void c(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        TouchSensor touchSensor = deviceManager.createDigitalTouchSensor(deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.touchSensor.put(deviceConfiguration.getName(), (Object)touchSensor);
    }

    private void d(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        AnalogInput analogInput = deviceManager.createAnalogInputDevice((AnalogInputController)deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.analogInput.put(deviceConfiguration.getName(), (Object)analogInput);
    }

    private void e(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        PWMOutput pWMOutput = deviceManager.createPwmOutputDevice(deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.pwmOutput.put(deviceConfiguration.getName(), (Object)pWMOutput);
    }

    private void f(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        I2cDevice i2cDevice = deviceManager.createI2cDevice((I2cController)deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.i2cDevice.put(deviceConfiguration.getName(), (Object)i2cDevice);
    }

    private void g(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        AnalogOutput analogOutput = deviceManager.createAnalogOutputDevice((AnalogOutputController)deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.analogOutput.put(deviceConfiguration.getName(), (Object)analogOutput);
    }

    private void h(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        OpticalDistanceSensor opticalDistanceSensor = deviceManager.createAnalogOpticalDistanceSensor(deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.opticalDistanceSensor.put(deviceConfiguration.getName(), (Object)opticalDistanceSensor);
    }

    private void a(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        TouchSensor touchSensor = deviceManager.createNxtTouchSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.touchSensor.put(deviceConfiguration.getName(), (Object)touchSensor);
    }

    private void b(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        TouchSensorMultiplexer touchSensorMultiplexer = deviceManager.createNxtTouchSensorMultiplexer(legacyModule, deviceConfiguration.getPort());
        hardwareMap.touchSensorMultiplexer.put(deviceConfiguration.getName(), (Object)touchSensorMultiplexer);
    }

    private void c(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        UltrasonicSensor ultrasonicSensor = deviceManager.createNxtUltrasonicSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.ultrasonicSensor.put(deviceConfiguration.getName(), (Object)ultrasonicSensor);
    }

    private void d(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        ColorSensor colorSensor = deviceManager.createNxtColorSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.colorSensor.put(deviceConfiguration.getName(), (Object)colorSensor);
    }

    private void e(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        GyroSensor gyroSensor = deviceManager.createNxtGyroSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.gyroSensor.put(deviceConfiguration.getName(), (Object)gyroSensor);
    }

    private void f(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        CompassSensor compassSensor = deviceManager.createNxtCompassSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.compassSensor.put(deviceConfiguration.getName(), (Object)compassSensor);
    }

    private void g(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        IrSeekerSensor irSeekerSensor = deviceManager.createNxtIrSeekerSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.irSeekerSensor.put(deviceConfiguration.getName(), (Object)irSeekerSensor);
    }

    private void h(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        LightSensor lightSensor = deviceManager.createNxtLightSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.lightSensor.put(deviceConfiguration.getName(), (Object)lightSensor);
    }

    private void i(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        AccelerationSensor accelerationSensor = deviceManager.createNxtAccelerationSensor(legacyModule, deviceConfiguration.getPort());
        hardwareMap.accelerationSensor.put(deviceConfiguration.getName(), (Object)accelerationSensor);
    }

    private void j(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        DcMotorController dcMotorController = deviceManager.createNxtDcMotorController(legacyModule, deviceConfiguration.getPort());
        hardwareMap.dcMotorController.put(deviceConfiguration.getName(), (Object)dcMotorController);
        for (DeviceConfiguration deviceConfiguration2 : ((MotorControllerConfiguration)deviceConfiguration).getMotors()) {
            DcMotor dcMotor = deviceManager.createDcMotor(dcMotorController, deviceConfiguration2.getPort());
            hardwareMap.dcMotor.put(deviceConfiguration2.getName(), (Object)dcMotor);
        }
    }

    private void k(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        ServoController servoController = deviceManager.createNxtServoController(legacyModule, deviceConfiguration.getPort());
        hardwareMap.servoController.put(deviceConfiguration.getName(), (Object)servoController);
        for (DeviceConfiguration deviceConfiguration2 : ((ServoControllerConfiguration)deviceConfiguration).getServos()) {
            Servo servo = deviceManager.createServo(servoController, deviceConfiguration2.getPort());
            hardwareMap.servo.put(deviceConfiguration2.getName(), (Object)servo);
        }
    }

    private void l(HardwareMap hardwareMap, DeviceManager deviceManager, LegacyModule legacyModule, DeviceConfiguration deviceConfiguration) {
        MatrixMasterController matrixMasterController = new MatrixMasterController((ModernRoboticsUsbLegacyModule)legacyModule, deviceConfiguration.getPort());
        MatrixDcMotorController matrixDcMotorController = new MatrixDcMotorController(matrixMasterController);
        hardwareMap.dcMotorController.put(deviceConfiguration.getName() + "Motor", (Object)matrixDcMotorController);
        hardwareMap.dcMotorController.put(deviceConfiguration.getName(), (Object)matrixDcMotorController);
        for (Object object : ((MatrixControllerConfiguration)deviceConfiguration).getMotors()) {
            DcMotor dcMotor = deviceManager.createDcMotor((DcMotorController)matrixDcMotorController, object.getPort());
            hardwareMap.dcMotor.put(object.getName(), (Object)dcMotor);
        }
        Object object2 = new MatrixServoController(matrixMasterController);
        hardwareMap.servoController.put(deviceConfiguration.getName() + "Servo", object2);
        hardwareMap.servoController.put(deviceConfiguration.getName(), object2);
        for (DcMotor dcMotor : ((MatrixControllerConfiguration)deviceConfiguration).getServos()) {
            Servo servo = deviceManager.createServo((ServoController)object2, dcMotor.getPort());
            hardwareMap.servo.put(dcMotor.getName(), (Object)servo);
        }
    }

    private void i(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        ColorSensor colorSensor = deviceManager.createAdafruitI2cColorSensor(deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.colorSensor.put(deviceConfiguration.getName(), (Object)colorSensor);
    }

    private void a(HardwareMap hardwareMap, DeviceManager deviceManager, DigitalChannelController digitalChannelController, DeviceConfiguration deviceConfiguration) {
        LED lED = deviceManager.createLED(digitalChannelController, deviceConfiguration.getPort());
        hardwareMap.led.put(deviceConfiguration.getName(), (Object)lED);
    }

    private void j(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        ColorSensor colorSensor = deviceManager.createModernRoboticsI2cColorSensor(deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.colorSensor.put(deviceConfiguration.getName(), (Object)colorSensor);
    }

    private void k(HardwareMap hardwareMap, DeviceManager deviceManager, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration deviceConfiguration) {
        GyroSensor gyroSensor = deviceManager.createModernRoboticsI2cGyroSensor(deviceInterfaceModule, deviceConfiguration.getPort());
        hardwareMap.gyroSensor.put(deviceConfiguration.getName(), (Object)gyroSensor);
    }

}

