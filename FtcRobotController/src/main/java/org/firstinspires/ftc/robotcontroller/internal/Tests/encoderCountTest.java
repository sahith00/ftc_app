package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by sahith on 11/2/17.
 */
public class encoderCountTest extends LinearOpMode{
    DcMotor frdrive, fldrive, brdrive, bldrive;

    final static double FORWARD_POWER = 0.28;
    final static int ENCODER_TICKS = 1920;
    ElapsedTime t = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fldrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bldrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fldrive.setPower(0.0);
        frdrive.setPower(0.0);
        bldrive.setPower(0.0);
        brdrive.setPower(0.0);



        waitForStart();

        sleep(1000);
        testMotorTicks2(frdrive, fldrive, brdrive, bldrive);
        sleep(22000);

        telemetry.update();
    }

    public void testMotorTicks(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        double t1, t2, t3, t4, t5, t6;
        motor1.setPower(0.8);
        motor2.setPower(-0.8);
        motor3.setPower(0.8);
        motor4.setPower(-0.8);
        while(motor1.getCurrentPosition() < 1000) {
            telemetry.addData("flmotore", fldrive.getCurrentPosition());
            telemetry.addData("frmotore", frdrive.getCurrentPosition());
            telemetry.addData("blmotore", bldrive.getCurrentPosition());
            telemetry.addData("brmotore", brdrive.getCurrentPosition());
            telemetry.addData("flmotorp", fldrive.getPower());
            telemetry.addData("frmotorp", frdrive.getPower());
            telemetry.addData("blmotorp", bldrive.getPower());
            telemetry.addData("brmotorp", brdrive.getPower());

            telemetry.update();
        }
        t1 = t.time();
        telemetry.addData("1st Time: ", t1);
        telemetry.addData("2nd Time: ", 0);
        telemetry.addData("3rd Time: ", 0);
        telemetry.addData("4th Time: ", 0);
        telemetry.addData("5th Time: ", 0);
        telemetry.addData("6th Time: ", 0);
        telemetry.update();
        motor1.setPower(0.6);
        motor2.setPower(-0.6);
        motor3.setPower(0.6);
        motor4.setPower(-0.6);
        while(motor1.getCurrentPosition() < 1500 && motor1.getCurrentPosition() >= 1000) {
            telemetry.addData("flmotore", fldrive.getCurrentPosition());
            telemetry.addData("frmotore", frdrive.getCurrentPosition());
            telemetry.addData("blmotore", bldrive.getCurrentPosition());
            telemetry.addData("brmotore", brdrive.getCurrentPosition());
            telemetry.addData("flmotorp", fldrive.getPower());
            telemetry.addData("frmotorp", frdrive.getPower());
            telemetry.addData("blmotorp", bldrive.getPower());
            telemetry.addData("brmotorp", brdrive.getPower());
            telemetry.update();
        }
        t2 = t.time();
        telemetry.addData("1st Time: ", t1);
        telemetry.addData("2nd Time: ", t2);
        telemetry.addData("3rd Time: ", 0);
        telemetry.addData("4th Time: ", 0);
        telemetry.addData("5th Time: ", 0);
        telemetry.addData("6th Time: ", 0);
        telemetry.update();
        motor1.setPower(0.5);
        motor2.setPower(-0.5);
        motor3.setPower(0.5);
        motor4.setPower(-0.5);
        while(motor1.getCurrentPosition() < 1750 && motor1.getCurrentPosition() >= 1500) {
            telemetry.addData("flmotore", fldrive.getCurrentPosition());
            telemetry.addData("frmotore", frdrive.getCurrentPosition());
            telemetry.addData("blmotore", bldrive.getCurrentPosition());
            telemetry.addData("brmotore", brdrive.getCurrentPosition());
            telemetry.addData("flmotorp", fldrive.getPower());
            telemetry.addData("frmotorp", frdrive.getPower());
            telemetry.addData("blmotorp", bldrive.getPower());
            telemetry.addData("brmotorp", brdrive.getPower());
            telemetry.update();
        }
        t3 = t.time();
        telemetry.addData("1st Time: ", t1);
        telemetry.addData("2nd Time: ", t2);
        telemetry.addData("3rd Time: ", t3);
        telemetry.addData("4th Time: ", 0);
        telemetry.addData("5th Time: ", 0);
        telemetry.addData("6th Time: ", 0);
        telemetry.update();
        motor1.setPower(0.3);
        motor2.setPower(-0.3);
        motor3.setPower(0.3);
        motor4.setPower(-0.3);
        while(motor1.getCurrentPosition() < 1880 && motor1.getCurrentPosition() >= 1750) {
            telemetry.addData("flmotore", fldrive.getCurrentPosition());
            telemetry.addData("frmotore", frdrive.getCurrentPosition());
            telemetry.addData("blmotore", bldrive.getCurrentPosition());
            telemetry.addData("brmotore", brdrive.getCurrentPosition());
            telemetry.addData("flmotorp", fldrive.getPower());
            telemetry.addData("frmotorp", frdrive.getPower());
            telemetry.addData("blmotorp", bldrive.getPower());
            telemetry.addData("brmotorp", brdrive.getPower());
            telemetry.update();
        }
        t4 = t.time();
        telemetry.addData("1st Time: ", t1);
        telemetry.addData("2nd Time: ", t2);
        telemetry.addData("3rd Time: ", t3);
        telemetry.addData("4th Time: ", t4);
        telemetry.addData("5th Time: ", 0);
        telemetry.addData("6th Time: ", 0);
        telemetry.update();
        motor1.setPower(0.2);
        motor2.setPower(-0.2);
        motor3.setPower(0.2);
        motor4.setPower(-0.2);
        while(motor1.getCurrentPosition() < 2000 && motor1.getCurrentPosition() >= 1880) {
            telemetry.addData("flmotore", fldrive.getCurrentPosition());
            telemetry.addData("frmotore", frdrive.getCurrentPosition());
            telemetry.addData("blmotore", bldrive.getCurrentPosition());
            telemetry.addData("brmotore", brdrive.getCurrentPosition());
            telemetry.addData("flmotorp", fldrive.getPower());
            telemetry.addData("frmotorp", frdrive.getPower());
            telemetry.addData("blmotorp", bldrive.getPower());
            telemetry.addData("brmotorp", brdrive.getPower());
            telemetry.update();
        }
        t5 = t.time();
        telemetry.addData("1st Time: ", t1);
        telemetry.addData("2nd Time: ", t2);
        telemetry.addData("3rd Time: ", t3);
        telemetry.addData("4th Time: ", t4);
        telemetry.addData("5th Time: ", t5);
        telemetry.addData("6th Time: ", 0);
        telemetry.update();
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        telemetry.addData("flmotore", fldrive.getCurrentPosition());
        telemetry.addData("frmotore", frdrive.getCurrentPosition());
        telemetry.addData("blmotore", bldrive.getCurrentPosition());
        telemetry.addData("brmotore", brdrive.getCurrentPosition());
        telemetry.addData("flmotorp", fldrive.getPower());
        telemetry.addData("frmotorp", frdrive.getPower());
        telemetry.addData("blmotorp", bldrive.getPower());
        telemetry.addData("brmotorp", brdrive.getPower());
        t6 = t.time();
        telemetry.addData("1st Time: ", t1);
        telemetry.addData("2nd Time: ", t2);
        telemetry.addData("3rd Time: ", t3);
        telemetry.addData("4th Time: ", t4);
        telemetry.addData("5th Time: ", t5);
        telemetry.addData("6th Time: ", t6);
        telemetry.update();
    }

    public void testMotorTicks2(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(FORWARD_POWER);
        motor2.setPower(-FORWARD_POWER);
        motor3.setPower(FORWARD_POWER);
        motor4.setPower(-FORWARD_POWER);
        while (motor1.getCurrentPosition() < ENCODER_TICKS) {
            telemetry.addData("flmotore", fldrive.getCurrentPosition());
            telemetry.addData("frmotore", frdrive.getCurrentPosition());
            telemetry.addData("blmotore", bldrive.getCurrentPosition());
            telemetry.addData("brmotore", brdrive.getCurrentPosition());
            telemetry.addData("flmotorp", fldrive.getPower());
            telemetry.addData("frmotorp", frdrive.getPower());
            telemetry.addData("blmotorp", bldrive.getPower());
            telemetry.addData("brmotorp", brdrive.getPower());
            telemetry.update();
        }
        motor1.setPower(0);
        motor2.setPower(-0);
        motor3.setPower(0);
        motor4.setPower(-0);
        telemetry.addData("flmotore", fldrive.getCurrentPosition());
        telemetry.addData("frmotore", frdrive.getCurrentPosition());
        telemetry.addData("blmotore", bldrive.getCurrentPosition());
        telemetry.addData("brmotore", brdrive.getCurrentPosition());
        telemetry.addData("flmotorp", fldrive.getPower());
        telemetry.addData("frmotorp", frdrive.getPower());
        telemetry.addData("blmotorp", bldrive.getPower());
        telemetry.addData("brmotorp", brdrive.getPower());
        telemetry.update();
    }

    public void testSideTicks(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        double joyly = 0;
        double joylx = -1;
        double joyrx = 0;
//        double bumpers = gamepad1.right_trigger-gamepad1.left_trigger;
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joyly, joylx);
        double v0 = joyrx;
        //v0 = bumpers;
        double v1 = vd*Math.sin(theta + (Math.PI / 4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))-v0; //fr
        double v3 = vd*Math.cos(theta + (Math.PI / 4))+v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));

        if (max != 0) {
            motor1.setPower(-0.8 * (v1 / max));
            motor2.setPower(0.8 * (v2 / max));
            motor3.setPower(-0.8 * (v3 / max));
            motor4.setPower(0.8 * (v4 / max));
            while(motor1.getCurrentPosition() < 200) {
            }
            motor1.setPower(-0.6 * (v1 / max));
            motor2.setPower(0.6 * (v2 / max));
            motor3.setPower(-0.6 * (v3 / max));
            motor4.setPower(0.6 * (v4 / max));
            while(motor1.getCurrentPosition() < 300) {
            }
            motor1.setPower(-0.5 * (v1 / max));
            motor2.setPower(0.5 * (v2 / max));
            motor3.setPower(-0.5 * (v3 / max));
            motor4.setPower(0.5 * (v4 / max));
            while(motor1.getCurrentPosition() < 350) {
            }
            motor1.setPower(-0.3 * (v1 / max));
            motor2.setPower(0.3 * (v2 / max));
            motor3.setPower(-0.3 * (v3 / max));
            motor4.setPower(0.3 * (v4 / max));
            while(motor1.getCurrentPosition() < 375) {
            }
            motor1.setPower(-0.2 * (v1 / max));
            motor2.setPower(0.2 * (v2 / max));
            motor3.setPower(-0.2 * (v3 / max));
            motor4.setPower(0.2 * (v4 / max));
            while(motor1.getCurrentPosition() < 390) {
            }
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }
    }

}
