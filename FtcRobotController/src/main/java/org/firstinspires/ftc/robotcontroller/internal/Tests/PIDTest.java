package org.firstinspires.ftc.robotcontroller.internal.Tests;


import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PIDTest extends LinearOpMode {
    DcMotor fr, fl, br, bl;

    ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    Orientation orientation;

    float roll, pitch, yaw;

    double temp;

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
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

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

        temp = degreeController(orientation.firstAngle);
        sleep(1000);
        turn(180, 5);
    }

    public void update(){
        roll = orientation.secondAngle;
        yaw = orientation.firstAngle;
        if (yaw > 180){
            yaw = -(360-yaw);
        }
        pitch = orientation.thirdAngle;
        Log.i("yaw", "" + yaw);
    }

    public double getDifference(double beg, double end) {
        if (end > beg) {
            if (Math.abs(end - beg) < Math.abs((end - 360) - beg)) {
                return end - beg;
            } else {
                return (end - 360) - beg;
            }
        } else if (end <= beg) {
            if (Math.abs(end - beg) < Math.abs((end + 360) - beg)) {
                return end - beg;
            } else {
                return (end + 360) - beg;
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

    public void startDegreeController() {
        pT = runtime.time();
        pE = 0;
        tE = 0;
    }

    public double degreeController(double degree) {
        double ans = 0;
        double e = Math.abs(getDifference(yaw, degree));
        double dE = e - pE;
        double dT = runtime.time() - pT;
        Log.i("PID turn", "e: " + e);
        Log.i("PID Turn", "i: " + i_turn * tE);
        Log.i("PID Turn", "d: " + d_turn * dE / dT);
        Log.i("PID Turn", "p: " + p_turn * e);
        ans = p_turn * e + i_turn * tE + d_turn * dE / dT;// +f_turn* Math.signum(e);
        pT = runtime.time();
        pE = e;
        tE += e * dT;
//        if (Math.signum(e) != Math.signum(pE)){
//            tE = 0;
//        }
        tE = Range.clip(tE * i_turn, -.15, 0.15);///i_turn;
        ans = Range.clip(ans, 0, .7);
        return ans;
    }

    public void turn(double degree, double margin) {
        startDegreeController();
        double pYaw = yaw;
        while (Math.abs(getDifference(yaw, degree)) > margin || Math.abs(pYaw - yaw) > .05) {
            Log.i("debug: ", "yaw: " + yaw);
            Log.i("debug: ", "difference: " + getDifference(yaw, degree));
            Log.i("debug", "target: " + degree + " cur: " + yaw);
            double change = degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            Log.i("powers", "forward: " + forwardPower);
            Log.i("powers", "backward: " + backPower);
            Log.i("debug: ", "difference: " + getDifference(yaw, degree));
            if (getDifference(yaw, degree) > 0) {
                fr.setPower(backPower);
                br.setPower(backPower);
                fl.setPower(forwardPower);
                bl.setPower(forwardPower);
            } else {
                fr.setPower(forwardPower);
                br.setPower(forwardPower);
                fl.setPower(backPower);
                bl.setPower(backPower);
            }
            Log.i("hey", "yaw: " + yaw);
            Log.i("degree", "degree: " + degree);
            pYaw = yaw;
            update();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
    }
}