package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.util.Log;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * Created by sahith on 12/10/17.
 * @see AdafruitBNO055IMU
 * @see BNO055IMU.Parameters
 */

public class revIMUTest extends LinearOpMode {
    DcMotor fr, fl, br, bl;

    BNO055IMU imu;
    Orientation lastAngles;

    ElapsedTime runtime = new ElapsedTime();

    int count = 0;
    double degrees[] = new double[7];

    @Override
    public void runOpMode() throws InterruptedException {
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        showPID();

        degrees[0] = 90;
        degrees[1] = -90;
        degrees[2] = 0;
        degrees[3] = 180;
        degrees[4] = 270;
        degrees[5] = 360;
        degrees[6] = -180;

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        while (opModeIsActive()) {
            showPID();
            telemetry.addData("last angles", formatAngle(AngleUnit.DEGREES, lastAngles.firstAngle));

            if (gamepad1.a) {
                if (count > 6) {
                    count = 0;
                }
                turn(degrees[count], 3);
                count += 1;
            }
            if (gamepad1.b) {
                if (count > 6) {
                    count = 0;
                }
                turn(degrees[count], 3);
                count += 2;
            }

            telemetry.addData("last angles", formatAngle(AngleUnit.DEGREES, lastAngles.firstAngle));
            telemetry.addData("finished", "done");
            telemetry.update();
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getDifference(double beg, double end){
        if (end > beg){
            if (Math.abs(end - beg) < Math.abs((end - 360) - beg)){
                return end - beg;
            } else{
                return  (end-360)-beg;
            }
        } else if(end <= beg){
            if (Math.abs(end - beg) < Math.abs((end + 360) - beg)){
                return end-beg;
            } else{
                return (end+360)-beg;
            }
        }
        return 0;
    }

    double p_turn = .03;//0.008;
    double i_turn = .00; //.0045; //.003;
    double d_turn = .002; //.04 //.0045;
    double f_turn = .0;
    double pT = 0;
    double pE = 0;
    double tE = 0;

    public void startDegreeController(){
        pT = runtime.time();
        pE = 0;
        tE = 0;
    }

    public double degreeController(double degree){
        double ans = 0;
        double e = Math.abs(getDifference(lastAngles.firstAngle, degree));
        double dE = e-pE;
        double dT = runtime.time() - pT;
        Log.i("PID turn", "e: " + e);
        Log.i("PID Turn", "i: " + i_turn * tE);
        Log.i("PID Turn", "d: " + d_turn * dE / dT);
        Log.i("PID Turn", "p: " + p_turn * e);
        ans = p_turn*e+ i_turn*tE + d_turn*dE/dT;// +f_turn* Math.signum(e);
        pT = runtime.time();
        pE = e;
        tE += e*dT;
//        if (Math.signum(e) != Math.signum(pE)){
//            tE = 0;
//        }
        tE = Range.clip(tE * i_turn, -.15, 0.15);///i_turn;
        ans = Range.clip(ans, 0, .7);
        return ans;
    }

    public void turn(double degree, double margin) {
        startDegreeController();
        double pYaw = lastAngles.firstAngle;
        while (Math.abs(getDifference(lastAngles.firstAngle, degree)) > margin || Math.abs(pYaw - lastAngles.firstAngle) > .05) {
            Log.i("debug: ", "yaw: " + lastAngles.firstAngle);
            Log.i("debug: ", "difference: " + getDifference(lastAngles.firstAngle, degree));
            Log.i("debug", "target: " + degree + " cur: " + lastAngles.firstAngle);
            double change = degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            Log.i("powers", "forward: " + forwardPower);
            Log.i("powers", "backward: " + backPower);
            Log.i("debug: ", "difference: " + getDifference(lastAngles.firstAngle, degree));
            if (getDifference(lastAngles.firstAngle, degree) > 0) {
                fr.setPower(0.5 * backPower);
                br.setPower(0.5 * backPower);
                fl.setPower(0.5 * forwardPower);
                bl.setPower(0.5 * forwardPower);
            } else {
                fr.setPower(0.5 * forwardPower);
                br.setPower(0.5 * forwardPower);
                fl.setPower(0.5 * backPower);
                bl.setPower(0.5 * backPower);
            }
            telemetry.addData("hey", "yaw: " + lastAngles.firstAngle);
            telemetry.addData("degree", "degree: " + degree);
            pYaw = lastAngles.firstAngle;
            resetAngles();
            telemetry.update();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        telemetry.addData("Reached", degree);
    }

    void resetAngles() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    void showPID() {
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.thirdAngle);
                    }
                });
    }
}
