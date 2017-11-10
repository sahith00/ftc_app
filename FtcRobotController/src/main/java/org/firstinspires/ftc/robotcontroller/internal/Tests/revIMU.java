package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.graphics.drawable.GradientDrawable;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by sahith on 11/9/17.
 */
public class revIMU extends LinearOpMode {
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    Orientation orientation;

    float roll, pitch, yaw;

    @Override
    public void runOpMode() throws InterruptedException {
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yaw = orientation.firstAngle;
        roll = orientation.secondAngle;
        pitch = orientation.thirdAngle;

        waitForStart();
    }

    public void update(){
        roll = orientation.secondAngle;
        //yaw = navx_device.getFusedHeading();
        if (yaw > 180){
            yaw = -(360-yaw);
        }
        pitch = orientation.thirdAngle;
        Log.i("yaw", "" + yaw);
    }
    public double getYaw() {
        return yaw;
    }
}
