/*package org.firstinspires.ftc.robotcontroller.opmodes.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.com.kauailabs.navx.ftc.*;


/**
 * Created by vulcanrobotics8375 on 11/30/16.
 */
/*public class NavX {
    private final int NAVX_DIM_I2C_PORT = 5;
    public AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    public double roll, yaw, pitch;

    public NavX(HardwareMap hM){
        navx_device = AHRS.getInstance(hM.deviceInterfaceModule.get("Device Interface Module 3"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
    }
    public void start(){ navx_device.zeroYaw(); }
    public void update(){
        roll = navx_device.getRoll();
        yaw = navx_device.getFusedHeading();
        if (yaw > 180){
            yaw = -(360-yaw);
        }
        pitch = navx_device.getPitch();
        Log.i("yaw", "" + yaw);
        Log.i("accelx", ""+navx_device.getWorldLinearAccelX());
        Log.i("accely", ""+navx_device.getWorldLinearAccelY());


    }
    public double getYaw() {
        return yaw;
    }
    public AHRS getNavx_device() { return navx_device; };
}*/