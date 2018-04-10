package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by sahith on 11/2/17.
 */
public class encoderCountTest extends LinearOpMode{
    DcMotor frdrive, fldrive, brdrive, bldrive;

    final static double FORWARD_POWER = 0.28;
    int ENCODER_TICKS = 1500;
    double maxpower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        brdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        fldrive.setPower(0.0);
        frdrive.setPower(0.0);
        bldrive.setPower(0.0);
        brdrive.setPower(0.0);
        brdrive.setTargetPosition(4);

        int change = 100;
        double powerchange = 0.05;



        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_down) {
                sleep(250);
                ENCODER_TICKS -= change;
            }
            if (gamepad1.dpad_up) {
                sleep(250);
                ENCODER_TICKS += change;
            }
            if (gamepad1.b) {
                sleep(250);
                ENCODER_TICKS = -ENCODER_TICKS;
                maxpower = -maxpower;
            }
            if(gamepad1.y) {
                sleep(250);
                maxpower+=powerchange;
                if(maxpower > 1.0) {
                    maxpower = 1.0;
                }
            }
            if(gamepad1.a) {
                sleep(250);
                maxpower-=powerchange;
                if(maxpower < 0.0) {
                    maxpower = 0.0;
                }
            }
            if (gamepad1.x) {
                sleep(1000);
                driveDistance(.5*ENCODER_TICKS, maxpower/1);
                driveDistance(.3*ENCODER_TICKS, maxpower/2);
                driveDistance(.2*ENCODER_TICKS, maxpower/3);
            }
            telemetry.addData("Encoder count:", bldrive.getCurrentPosition());
            telemetry.addData("Encoder ticks:", ENCODER_TICKS);
            telemetry.addData("power", maxpower);
            telemetry.update();
        }

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

    public void driveDistance(double distance, double maxpower) {
        if (distance > 0) {
            int ticks;
            int old_ticks = bldrive.getCurrentPosition();
            ticks = (int) (distance);
            frdrive.setPower(maxpower);
            fldrive.setPower(maxpower);
            brdrive.setPower(maxpower);
            bldrive.setPower(maxpower);
            while (bldrive.getCurrentPosition() < (ticks + old_ticks)) {
            }
            frdrive.setPower(0);
            fldrive.setPower(0);
            brdrive.setPower(0);
            bldrive.setPower(0);
            telemetry.addData("Encoder count:", bldrive.getCurrentPosition());
            telemetry.addData("Encoder ticks:", ENCODER_TICKS);
            telemetry.update();
        }
        else if (distance < 0) {
            int ticks;
            int old_ticks = bldrive.getCurrentPosition();
            ticks = (int) (distance);
            frdrive.setPower(maxpower);
            fldrive.setPower(maxpower);
            brdrive.setPower(maxpower);
            bldrive.setPower(maxpower);
            while (bldrive.getCurrentPosition() > (ticks + old_ticks)) {
            }
            frdrive.setPower(0);
            fldrive.setPower(0);
            brdrive.setPower(0);
            bldrive.setPower(0);
            telemetry.addData("Encoder count:", bldrive.getCurrentPosition());
            telemetry.addData("Encoder ticks:", ENCODER_TICKS);
            telemetry.update();
        }
    }

}
