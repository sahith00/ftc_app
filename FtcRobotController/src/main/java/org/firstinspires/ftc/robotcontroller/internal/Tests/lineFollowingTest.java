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

public class lineFollowingTest extends LinearOpMode {
    ColorSensor lineSensor;
    DcMotor fr, fl, br, bl;
    final static int DESIREDBLUE = 23;
    int error;
    double leftPower, rightPower, addedPower;

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
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);

        while (!isStarted()) {
            telemetry.update();
            idle();
        }
        while (opModeIsActive()) {
            setBack(0.0);
            error = DESIREDBLUE - lineSensor.blue();
            addedPower = Range.clip(convertColorToPower(error), -0.075, 0.075);
            if (addedPower <= 0) {
                leftPower = 0.075 - addedPower;
                rightPower = 0.075;
            }
            else {
                leftPower = 0.075;
                rightPower = 0.075 + addedPower;
            }
            fl.setPower(leftPower);
            fr.setPower(rightPower);

            telemetry.addData("fr", fr.getPower());
            telemetry.addData("fl", fl.getPower());
            telemetry.addData("br", br.getPower());
            telemetry.addData("bl", bl.getPower());
            telemetry.addData("blue", lineSensor.blue());
            telemetry.addData("error", addedPower);
            telemetry.update();
        }
    }
    public void setForward(double lpower) {
        fl.setPower(lpower);
        fr.setPower(lpower);
    }
    public void setBack(double rpower) {
        bl.setPower(rpower);
        br.setPower(rpower);
    }

    public double convertColorToPower(int color) {
        double temp = (double) (color);
        return .00441176 * temp;
    }
}
