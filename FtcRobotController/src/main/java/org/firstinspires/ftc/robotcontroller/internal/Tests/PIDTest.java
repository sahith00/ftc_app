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

    NavX navX;

    final static int MARGIN = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        navX = new NavX(hardwareMap);
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
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        navX.start();
        waitForStart();

        telemetry.addData("initial yaw:", navX.getYaw());
        sleep(1000);
        turn(90, MARGIN);
        telemetry.addData("final yaw:", navX.getYaw());

        while (opModeIsActive()) {
            if(gamepad1.a) {
                turn(90, MARGIN);
            }
            if(gamepad1.b) {
                turn(-90, MARGIN);
            }
            if(gamepad1.x) {
                turn(180, MARGIN);
            }
            if(gamepad1.y) {
                turn(0, MARGIN);
            }
            telemetry.addData("yaw", navX.getYaw());
            telemetry.update();
        }
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

    public double degreeController(double degree, NavX navX){
        double ans = 0;
        double e = Math.abs(getDifference(navX.getYaw(), degree));
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
        double pYaw = navX.yaw;
        while (Math.abs(getDifference(navX.yaw, degree)) > margin || Math.abs(pYaw - navX.yaw) > .05) {
            Log.i("debug: ", "yaw: " + navX.yaw);
            Log.i("debug: ", "difference: " + getDifference(navX.yaw, degree));
            Log.i("debug", "target: " + degree + " cur: " + navX.yaw);
            double change = degreeController(degree, navX);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            Log.i("powers", "forward: " + forwardPower);
            Log.i("powers", "backward: " + backPower);
            Log.i("debug: ", "difference: " + getDifference(navX.yaw, degree));
            if (getDifference(navX.yaw, degree) > 0) {
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
            Log.i("hey", "yaw: " + navX.yaw);
            Log.i("degree", "degree: " + degree);
            pYaw = navX.yaw;
            navX.update();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
    }
}