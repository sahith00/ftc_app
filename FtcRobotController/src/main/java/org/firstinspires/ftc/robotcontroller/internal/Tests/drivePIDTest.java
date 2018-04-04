package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sahith on 4/3/18.
 */

public class drivePIDTest extends LinearOpMode {
    DcMotor fr, fl, br, bl;
    double p = .0045, i = .002, d = .002;
    double pE = 0, pT = 0, tE = 0;
    double change = .0001;
    double[] changes;
    int count = 0;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        changes[0] = .0001;
        changes[1] = .001;
        changes[2] = .01;

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpad_down) {
                sleep(250);
                p -= change;
            }
            if (gamepad1.dpad_up) {
                sleep(250);
                p += change;
            }
            if(gamepad1.right_bumper) {
                sleep(250);
                i -= change;
            }
            if (gamepad1.left_bumper) {
                sleep(250);
                i += change;
            }
            if(gamepad1.a) {
                sleep(250);
                d -= change;
            }
            if (gamepad1.y) {
                sleep(250);
                d += change;
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
                sleep(1000);
                drivePID(2000);
            }

            telemetry.addData("p", p);
            telemetry.addData("i", i);
            telemetry.addData("d", d);
            telemetry.addData("fl position", fl.getCurrentPosition());
            telemetry.update();
        }

    }

    public double degreeController(double position){
        double ans = 0;
        double e = Math.abs(fl.getCurrentPosition() - position);
        double dE = e-pE;
        double dT = runtime.time() - pT;
        ans = p*e+ i*tE + d*dE/dT;//+f_turn* Math.signum(e);
        pT = runtime.time();
        pE = e;
        tE += e*dT;
        tE = Range.clip(tE * i, -.15, 0.15);///i_turn;
        ans = Range.clip(ans, 0, .7);
        return ans;
    }

    public void drivePID(double position) {

    }

}
