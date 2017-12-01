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
    int ENCODER_TICKS = 1700;
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
        fldrive.setPower(0.0);
        frdrive.setPower(0.0);
        bldrive.setPower(0.0);
        brdrive.setPower(0.0);

        int change = 10;



        waitForStart();

        sleep(3000);
        testSideTicks(frdrive, fldrive, brdrive, bldrive);
        sleep(1000);
        while (opModeIsActive()) {
            if(gamepad1.dpad_down) {
                sleep(250);
                ENCODER_TICKS -= change;
            }
            if (gamepad1.dpad_up) {
                sleep(250);
                ENCODER_TICKS += change;
            }
            if (gamepad1.a) {
                sleep(1000);
                testMotorTicks(frdrive, fldrive, brdrive, bldrive);
            }
        }

        telemetry.update();
    }

    public void testMotorTicks(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        double multi, old_multi;
        motor1.setPower(0.2);
        motor2.setPower(-0.2);
        motor3.setPower(0.2);
        motor4.setPower(-0.2);
        multi = 0.1;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi)) {
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
        motor1.setPower(0.35);
        motor2.setPower(-0.35);
        motor3.setPower(0.35);
        motor4.setPower(-0.35);
        old_multi = multi;
        multi = 0.2;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        motor1.setPower(0.5);
        motor2.setPower(-0.5);
        motor3.setPower(0.5);
        motor4.setPower(-0.5);
        old_multi = multi;
        multi = 0.3;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        motor1.setPower(0.65);
        motor2.setPower(-0.65);
        motor3.setPower(0.65);
        motor4.setPower(-0.65);
        old_multi = multi;
        multi = 0.4;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        motor1.setPower(0.8);
        motor2.setPower(-0.8);
        motor3.setPower(0.8);
        motor4.setPower(-0.8);
        old_multi = multi;
        multi = 0.5;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        motor1.setPower(0.65);
        motor2.setPower(-0.65);
        motor3.setPower(0.65);
        motor4.setPower(-0.65);
        old_multi = multi;
        multi = 0.6;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        motor1.setPower(0.5);
        motor2.setPower(-0.5);
        motor3.setPower(0.5);
        motor4.setPower(-0.5);
        old_multi = multi;
        multi = 0.7;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        motor1.setPower(0.35);
        motor2.setPower(-0.35);
        motor3.setPower(0.35);
        motor4.setPower(-0.35);
        old_multi = multi;
        multi = 0.85;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        motor1.setPower(0.2);
        motor2.setPower(-0.2);
        motor3.setPower(0.2);
        motor4.setPower(-0.2);
        old_multi = multi;
        multi = 1.0;
        while(motor1.getCurrentPosition() < (ENCODER_TICKS * multi) && motor1.getCurrentPosition() >= (ENCODER_TICKS * old_multi)) {
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
        double multi, old_multi;
        motor1.setPower(0.8);
        motor2.setPower(0.8);
        motor3.setPower(-0.8);
        motor4.setPower(-0.8);
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
        telemetry.update();
    }

}
