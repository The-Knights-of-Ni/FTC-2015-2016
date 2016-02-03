/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class DcMotor
implements HardwareDevice {
    protected Direction direction = Direction.FORWARD;
    protected DcMotorController controller = null;
    protected int portNumber = -1;
    protected DcMotorController.RunMode mode = DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
    protected DcMotorController.DeviceMode devMode = DcMotorController.DeviceMode.WRITE_ONLY;

    public DcMotor(DcMotorController controller, int portNumber) {
        this(controller, portNumber, Direction.FORWARD);
    }

    public DcMotor(DcMotorController controller, int portNumber, Direction direction) {
        this.controller = controller;
        this.portNumber = portNumber;
        this.direction = direction;
    }

    @Override
    public String getDeviceName() {
        return "DC Motor";
    }

    @Override
    public String getConnectionInfo() {
        return this.controller.getConnectionInfo() + "; port " + this.portNumber;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void close() {
        this.setPowerFloat();
    }

    public DcMotorController getController() {
        return this.controller;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public Direction getDirection() {
        return this.direction;
    }

    public int getPortNumber() {
        return this.portNumber;
    }

    public void setPower(double power) {
        if (this.direction == Direction.REVERSE) {
            power*=-1.0;
        }
        if (this.mode == DcMotorController.RunMode.RUN_TO_POSITION) {
            power = Math.abs(power);
        }
        this.controller.setMotorPower(this.portNumber, power);
    }

    public double getPower() {
        double d = this.controller.getMotorPower(this.portNumber);
        if (this.direction == Direction.REVERSE && d != 0.0) {
            d*=-1.0;
        }
        return d;
    }

    public boolean isBusy() {
        return this.controller.isBusy(this.portNumber);
    }

    public void setPowerFloat() {
        this.controller.setMotorPowerFloat(this.portNumber);
    }

    public boolean getPowerFloat() {
        return this.controller.getMotorPowerFloat(this.portNumber);
    }

    public void setTargetPosition(int position) {
        if (this.direction == Direction.REVERSE) {
            position*=-1;
        }
        this.controller.setMotorTargetPosition(this.portNumber, position);
    }

    public int getTargetPosition() {
        int n = this.controller.getMotorTargetPosition(this.portNumber);
        if (this.direction == Direction.REVERSE) {
            n*=-1;
        }
        return n;
    }

    public int getCurrentPosition() {
        int n = this.controller.getMotorCurrentPosition(this.portNumber);
        if (this.direction == Direction.REVERSE) {
            n*=-1;
        }
        return n;
    }

    public void setMode(DcMotorController.RunMode mode) {
        this.mode = mode;
        this.controller.setMotorChannelMode(this.portNumber, mode);
    }

    public DcMotorController.RunMode getMode() {
        return this.controller.getMotorChannelMode(this.portNumber);
    }

    @Deprecated
    public void setChannelMode(DcMotorController.RunMode mode) {
        this.setMode(mode);
    }

    @Deprecated
    public DcMotorController.RunMode getChannelMode() {
        return this.getMode();
    }

    public static enum Direction {
        FORWARD,
        REVERSE;
        

        private Direction() {
        }
    }

}

