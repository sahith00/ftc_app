package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
/**
 * Created by sahith on 1/3/18.
 */

public class BasicLineFollowing extends LinearOpMode {
    ColorSensor lineSensor;
    DcMotor fr, fl, br, bl;
    int desiredblue, currentblue, counter;

    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        lineSensor = hardwareMap.colorSensor.get("lineSensor");
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        desiredblue = 23;
        currentblue = 6;
        counter = 0;

        while (!isStarted()) {
            telemetry.update();
            idle();
        }
        while (opModeIsActive()) {
            if (lineSensor.blue() > desiredblue) {
                setRight(.09);
                setLeft(.0);
            } else {
                setRight(.0);
                setLeft(.09);
            }
        }
    }
    public void setLeft(double lpower) {
        fl.setPower(lpower);
        bl.setPower(lpower);
    }
    public void setRight(double rpower) {
        fr.setPower(rpower);
        br.setPower(rpower);
    }
}
