package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sahith on 4/3/18.
 */

public class drivePIDTest extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles;

    DcMotor fr, fl, br, bl;
    double p_turn = .052, i_turn = .002, d_turn = .002;
    double pE = 0, pT = 0, tE = 0;
    double change = .0001;
    double marginchange = 0.5;
    double margin = 1.5;
    double[] changes = new double[3];
    double[] degrees = new double[4];
    int count = 0;
    int degreecount = 0;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        changes[0] = .0001;
        changes[1] = .001;
        changes[2] = .01;

        degrees[0] = 0;
        degrees[1] = 90;
        degrees[2] = 180;
        degrees[3] = 270;

        resetAngles();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpad_down) {
                sleep(250);
                p_turn -= change;
            }
            if (gamepad1.dpad_up) {
                sleep(250);
                p_turn += change;
            }
            if(gamepad1.right_bumper) {
                sleep(250);
                i_turn -= change;
            }
            if (gamepad1.left_bumper) {
                sleep(250);
                i_turn += change;
            }
            if(gamepad1.a) {
                sleep(250);
                d_turn -= change;
            }
            if (gamepad1.y) {
                sleep(250);
                d_turn += change;
            }
            if(gamepad1.dpad_left) {
                sleep(250);
                margin -= marginchange;
                if(margin < 0.5) {
                    margin = 0.5;
                }
            }
            if(gamepad1.dpad_right) {
                sleep(250);
                margin += marginchange;
                if(margin > 5) {
                    margin = 5;
                }
            }
            if(gamepad1.b) {
                sleep(250);
                count += 1;
                if(count > 2) {
                    count = 0;
                }
                change = changes[count];
            }
            if(gamepad1.x) {
                sleep(250);
                degreecount+=1;
                if(degreecount > 3) {
                    degreecount = 0;
                }
                sleep(1000);
                turn(degrees[degreecount], margin);
            }

            telemetry.addData("p", p_turn);
            telemetry.addData("i", i_turn);
            telemetry.addData("d", d_turn);
            telemetry.addData("change", change);
            telemetry.addData("margin", margin);
            telemetry.update();
        }
    }

    public void turn(double degree, double margin) {
        startDegreeController();
        double pYaw = lastAngles.firstAngle;
        while ((Math.abs(getDifference(lastAngles.firstAngle, degree)) > margin ||
                Math.abs(pYaw - lastAngles.firstAngle) > .05) && opModeIsActive()) {
            double change = degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            if (getDifference(lastAngles.firstAngle, degree) > 0) {
                fr.setPower(0.8 * backPower);
                br.setPower(0.8 * backPower);
                fl.setPower(0.8 * forwardPower);
                bl.setPower(0.8 * forwardPower);
            } else {
                fr.setPower(0.8 * forwardPower);
                br.setPower(0.8 * forwardPower);
                fl.setPower(0.8 * backPower);
                bl.setPower(0.8 * backPower);
            }
            pYaw = lastAngles.firstAngle;
            resetAngles();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
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

    public void resetAngles() {
        lastAngles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

}
