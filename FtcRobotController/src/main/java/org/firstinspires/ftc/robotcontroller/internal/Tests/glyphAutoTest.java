package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by sahith on 1/19/18.
 */

public class glyphAutoTest extends LinearOpMode {
    DcMotor fr, fl, br, bl;
    DcMotor rgrab, lgrab;
    Servo rflip, lflip;

    BNO055IMU imu;
    Orientation lastAngles;

    final static double STRAIGHT_TICKS_PER_INCH = 49.23;
    final static double SIDE_TICKS_PER_INCH = 51.51;

    final static double RFLIP_DEPOSIT = 0.215; //0.47
    final static double RFLIP_ZERO = 0.505; //0.74
    final static double RFLIP_GRAB = 0.575;
    final static double LFLIP_DEPOSIT = 0.87995; //0.53
    final static double LFLIP_ZERO = 0.3495; //0.26
    final static double LFLIP_GRAB = 0.2395;

    double p_turn = .03;//0.008;
    double i_turn = .00; //.0045; //.003;
    double d_turn = .002; //.04 //.0045;
    double f_turn = .0;
    double pT = 0;
    double pE = 0;
    double tE = 0;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        rgrab = hardwareMap.dcMotor.get("rintake");
        lgrab = hardwareMap.dcMotor.get("lintake");

        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        rgrab.setPower(0);
        lgrab.setPower(0);

        grab();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        showPID();

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        turn(90, 3);
        grabGlyph(-0.7);
        drive(15, "FORWARD", 0.8);
        grabGlyph(0);
        drive(5, "BACKWARD", 0.8);
        turn(-90, 3);
        outtake();
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

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
        ans = p_turn*e+ i_turn*tE + d_turn*dE/dT;//+f_turn* Math.signum(e);
        pT = runtime.time();
        pE = e;
        tE += e*dT;
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


    public void grabGlyph(double power) {
        rgrab.setPower(power);
        lgrab.setPower(power);
    }

    public void outtake() {
        zero();
        sleep(500);
        deposit();
        sleep(500);
        zero();
        sleep(500);
    }

    public void grab() {
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
    }
    public void zero() {
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
    }
    public void deposit() {
        rflip.setPosition(RFLIP_DEPOSIT);
        lflip.setPosition(LFLIP_DEPOSIT);
    }

    public void drive(double distance, String direction, double maxpower) {
        int ticks;
        int old_ticks = fr.getCurrentPosition();
        double multi, old_multi;
        switch (direction) {
            case "FORWARD":
                ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
                multi = 0.15;
                old_multi = 0.15;
                fr.setPower(0.3 * maxpower);
                fl.setPower(0.3 * maxpower);
                br.setPower(0.3 * maxpower);
                bl.setPower(0.3 * maxpower);
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.5 * maxpower);
                fl.setPower(0.5 * maxpower);
                br.setPower(0.5 * maxpower);
                bl.setPower(0.5 * maxpower);
                multi = 0.3;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.75 * maxpower);
                fl.setPower(0.75 * maxpower);
                br.setPower(0.75 * maxpower);
                bl.setPower(0.75 * maxpower);
                old_multi = multi;
                multi = 0.45;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(1 * maxpower);
                fl.setPower(1 * maxpower);
                br.setPower(1 * maxpower);
                bl.setPower(1 * maxpower);
                old_multi = multi;
                multi = 0.6;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.75 * maxpower);
                fl.setPower(0.75 * maxpower);
                br.setPower(0.75 * maxpower);
                bl.setPower(0.75 * maxpower);
                old_multi = multi;
                multi = 0.75;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.5 * maxpower);
                fl.setPower(0.5 * maxpower);
                br.setPower(0.5 * maxpower);
                bl.setPower(0.5 * maxpower);
                old_multi = multi;
                multi = 0.85;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.3 * maxpower);
                fl.setPower(0.3 * maxpower);
                br.setPower(0.3 * maxpower);
                bl.setPower(0.3 * maxpower);
                old_multi = multi;
                multi = 1;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                bl.setPower(0);
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
                break;
            case "RIGHT":
                ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
                multi = 0.15;
                old_multi = 0.15;
                fr.setPower(-0.3 * maxpower);
                fl.setPower(0.3 * maxpower);
                br.setPower(0.3 * maxpower);
                bl.setPower(-0.3 * maxpower);
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.5 * maxpower);
                fl.setPower(0.5 * maxpower);
                br.setPower(0.5 * maxpower);
                bl.setPower(-0.5 * maxpower);
                multi = 0.3;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.75 * maxpower);
                fl.setPower(0.75 * maxpower);
                br.setPower(0.75 * maxpower);
                bl.setPower(-0.75 * maxpower);
                old_multi = multi;
                multi = 0.45;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-1 * maxpower);
                fl.setPower(1 * maxpower);
                br.setPower(1 * maxpower);
                bl.setPower(-1 * maxpower);
                old_multi = multi;
                multi = 0.6;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.75 * maxpower);
                fl.setPower(0.75 * maxpower);
                br.setPower(0.75 * maxpower);
                bl.setPower(-0.75 * maxpower);
                old_multi = multi;
                multi = 0.75;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.5 * maxpower);
                fl.setPower(0.5 * maxpower);
                br.setPower(0.5 * maxpower);
                bl.setPower(-0.5 * maxpower);
                old_multi = multi;
                multi = 0.85;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.3 * maxpower);
                fl.setPower(0.3 * maxpower);
                br.setPower(0.3 * maxpower);
                bl.setPower(-0.3 * maxpower);
                old_multi = multi;
                multi = 1;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                bl.setPower(0);
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
                break;
            case "BACKWARD":
                ticks = (int) (SIDE_TICKS_PER_INCH * distance);
                multi = 0.15;
                old_multi = 0.15;
                fr.setPower(-0.3 * maxpower);
                fl.setPower(-0.3 * maxpower);
                br.setPower(-0.3 * maxpower);
                bl.setPower(-0.3 * maxpower);
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.5 * maxpower);
                fl.setPower(-0.5 * maxpower);
                br.setPower(-0.5 * maxpower);
                bl.setPower(-0.5 * maxpower);
                multi = 0.3;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.75 * maxpower);
                fl.setPower(-0.75 * maxpower);
                br.setPower(-0.75 * maxpower);
                bl.setPower(-0.75 * maxpower);
                old_multi = multi;
                multi = 0.45;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-1 * maxpower);
                fl.setPower(-1 * maxpower);
                br.setPower(-1 * maxpower);
                bl.setPower(-1 * maxpower);
                old_multi = multi;
                multi = 0.6;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.75 * maxpower);
                fl.setPower(-0.75 * maxpower);
                br.setPower(-0.75 * maxpower);
                bl.setPower(-0.75 * maxpower);
                old_multi = multi;
                multi = 0.75;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.5 * maxpower);
                fl.setPower(-0.5 * maxpower);
                br.setPower(-0.5 * maxpower);
                bl.setPower(-0.5 * maxpower);
                old_multi = multi;
                multi = 0.85;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(-0.3 * maxpower);
                fl.setPower(-0.3 * maxpower);
                br.setPower(-0.3 * maxpower);
                bl.setPower(-0.3 * maxpower);
                old_multi = multi;
                multi = 1;
                while (fr.getCurrentPosition() > (-(ticks + old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                bl.setPower(0);
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
                break;
            case "LEFT":
                ticks = (int) (SIDE_TICKS_PER_INCH * distance);
                multi = 0.15;
                old_multi = 0.15;
                fr.setPower(0.3 * maxpower);
                fl.setPower(-0.3 * maxpower);
                br.setPower(-0.3 * maxpower);
                bl.setPower(0.3 * maxpower);
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.5 * maxpower);
                fl.setPower(-0.5 * maxpower);
                br.setPower(-0.5 * maxpower);
                bl.setPower(0.5 * maxpower);
                multi = 0.3;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.75 * maxpower);
                fl.setPower(-0.75 * maxpower);
                br.setPower(-0.75 * maxpower);
                bl.setPower(0.75 * maxpower);
                old_multi = multi;
                multi = 0.45;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(1 * maxpower);
                fl.setPower(-1 * maxpower);
                br.setPower(-1 * maxpower);
                bl.setPower(1 * maxpower);
                old_multi = multi;
                multi = 0.6;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.75 * maxpower);
                fl.setPower(-0.75 * maxpower);
                br.setPower(-0.75 * maxpower);
                bl.setPower(0.75 * maxpower);
                old_multi = multi;
                multi = 0.75;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.5 * maxpower);
                fl.setPower(-0.5 * maxpower);
                br.setPower(-0.5 * maxpower);
                bl.setPower(0.5 * maxpower);
                old_multi = multi;
                multi = 0.85;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0.3 * maxpower);
                fl.setPower(-0.3 * maxpower);
                br.setPower(-0.3 * maxpower);
                bl.setPower(0.3 * maxpower);
                old_multi = multi;
                multi = 1;
                while (fr.getCurrentPosition() < ((ticks + old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks + old_ticks) * old_multi)) {
                    telemetry.addData("flmotore", fl.getCurrentPosition());
                    telemetry.addData("frmotore", fr.getCurrentPosition());
                    telemetry.addData("blmotore", bl.getCurrentPosition());
                    telemetry.addData("brmotore", br.getCurrentPosition());
                    telemetry.addData("flmotorp", fl.getPower());
                    telemetry.addData("frmotorp", fr.getPower());
                    telemetry.addData("blmotorp", bl.getPower());
                    telemetry.addData("brmotorp", br.getPower());
                    telemetry.update();
                }
                fr.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                bl.setPower(0);
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
                break;
        }
    }
}