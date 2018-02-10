package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by sahith on 2/5/18.
 */

public class Drivetrain {
    public DcMotor fr, fl, br, bl;
    public PID pid;

    final static double STRAIGHT_TICKS_PER_INCH = 52.63;
    final static double SIDE_TICKS_PER_INCH = 51.51;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.fr = hardwareMap.dcMotor.get("frdrive");
        this.fl = hardwareMap.dcMotor.get("fldrive");
        this.br = hardwareMap.dcMotor.get("brdrive");
        this.bl = hardwareMap.dcMotor.get("bldrive");
        pid = new PID(hardwareMap, telemetry);
    }

    public void driveForward(double distance, double maxpower) {
        int ticks;
        int old_ticks = fl.getCurrentPosition();
        ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
        fr.setPower(maxpower);
        fl.setPower(maxpower);
        br.setPower(maxpower);
        bl.setPower(maxpower);
        while (fl.getCurrentPosition() < ticks + old_ticks) {
        }
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void driveBackward(double distance, double maxpower) {
        int ticks;
        int old_ticks = fl.getCurrentPosition();
        ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
        fr.setPower(maxpower);
        fl.setPower(maxpower);
        br.setPower(maxpower);
        bl.setPower(maxpower);
        while (fl.getCurrentPosition() > ticks + old_ticks) {
        }
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void mecanum(double joyly, double joylx, double joyrx, double multiplier) {
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joyly, joylx);
        double v0 = joyrx;
        //v0 = bumpers;
        double v1 = vd*Math.sin(theta+(Math.PI/4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))+v0; //fr
        double v3 = vd*Math.cos(theta+(Math.PI/4))-v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));
        if (max > 1) {
            fl.setPower(multiplier * (v1/max));
            fr.setPower(multiplier * (v2/max));
            bl.setPower(multiplier * (v3/max));
            br.setPower(multiplier * (v4/max));
        } else {
            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);
        }
    }

    public void turn(double degree, double margin) {
        pid.startDegreeController();
        double pYaw = pid.lastAngles.firstAngle;
        while (Math.abs(pid.getDifference(pid.lastAngles.firstAngle, degree)) > margin ||
                Math.abs(pYaw - pid.lastAngles.firstAngle) > .05) {
            double change = pid.degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            if (pid.getDifference(pid.lastAngles.firstAngle, degree) > 0) {
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
            pYaw = pid.lastAngles.firstAngle;
            pid.resetAngles();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
    }

    public void showPID() {
        pid.showPID();
    }
}
