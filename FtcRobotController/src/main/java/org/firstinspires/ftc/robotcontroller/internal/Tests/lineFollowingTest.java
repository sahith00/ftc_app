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
 * Created by sahith on 12/10/17.
 */

public class lineFollowingTest extends LinearOpMode {
    ColorSensor lineSensor;
    DcMotor fr, fl, br, bl;
    int firstBlue;
    int[] firstBlueRange, notBlueRange;
    double firstAngle;

    BNO055IMU imu;
    Orientation lastAngles;

    ElapsedTime runtime = new ElapsedTime();

    @Override
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        showPID();
        firstBlue = lineSensor.blue();
        firstBlueRange = new int[5];
        firstBlueRange[0] = firstBlue-2;
        firstBlueRange[1] = firstBlue-1;
        firstBlueRange[2] = firstBlue;
        firstBlueRange[3] = firstBlue+1;
        firstBlueRange[4] = firstBlue+2;
        notBlueRange = new int[7];
        notBlueRange[0] = 2;
        notBlueRange[1] = 3;
        notBlueRange[2] = 4;
        notBlueRange[3] = 5;
        notBlueRange[4] = 6;
        notBlueRange[5] = 7;
        notBlueRange[6] = 8;


        telemetry.addData("First Blue", firstBlue);

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        while(opModeIsActive()) {
            int n = 0;
            resetAngles();
            firstAngle = lastAngles.firstAngle;
            while (n < 100) {            //change to distance-based
                followLine();
                n++;
                telemetry.addData("dsfhai", "Did follow line");
                telemetry.update();
            }
            telemetry.addData("dsfhai", "Did follow line 100 times");
            fl.setPower(0.0);
            fr.setPower(0.0);
            bl.setPower(0.0);
            br.setPower(0.0);

            telemetry.update();
        }
    }

    public void followLine() {
        if (listIncludesElement(firstBlueRange, lineSensor.blue())) {
            fl.setPower(0.1);
            fr.setPower(0.1);
            bl.setPower(0.1);
            br.setPower(0.1);
            telemetry.addData("HI", "Correct blue detected");
            telemetry.update();
        }
        while (lineSensor.blue() > firstBlueRange[4] + 3) {
            turn(5, 0.5);
            telemetry.addData("HI", "Turned 5");
            telemetry.update();
            sleep(1000);
            //followLine();
        }
        while (lineSensor.blue() < firstBlueRange[0] - 3) {
            turn(-5, 0.5);
            telemetry.addData("HI", "Turned -5");
            telemetry.update();
            sleep(1000);
            //followLine();
        }
        telemetry.update();
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
            double change = degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
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
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        telemetry.addData("Reached", degree);
        telemetry.update();
    }

    void resetAngles() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastAngles.firstAngle = 0;
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

    public boolean listIncludesElement(int[] l, int x) {
        for (int i = 0; i < l.length; i++) {
            if (x == l[i]) {
                return true;
            }
        }
        return false;
    }
}
