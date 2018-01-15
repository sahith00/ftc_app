package org.firstinspires.ftc.robotcontroller.internal.Cisco;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sahith on 12/10/17.
 */

public class ciscoTeleOp extends LinearOpMode{

    DcMotor frdrive, fldrive, brdrive, bldrive;
    DcMotor rgrab, lgrab;
    DcMotor lift;
    Servo flipper;
    //Servo cat, knock;

    final static double CAT_STOW = 0.66;
    final static double KNOCK_CENTER = 0.38;

    int direction = 1;

    boolean stopped;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        lift = hardwareMap.dcMotor.get("lift");
        rgrab = hardwareMap.dcMotor.get("rintake");
        lgrab = hardwareMap.dcMotor.get("lintake");
        flipper = hardwareMap.servo.get("flipper");
        //cat = hardwareMap.servo.get("cat");
        //knock = hardwareMap.servo.get("knock");

        fldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        //lift.setPower(0);
        rgrab.setPower(0);
        lgrab.setPower(0);
        //cat.setPosition(CAT_STOW);
        //knock.setPosition(KNOCK_CENTER);

        double multiplier = 1.0;

        waitForStart();

        while(opModeIsActive()) {
            //cat.setPosition(CAT_STOW);
            //knock.setPosition(KNOCK_CENTER);

            //-----------------------------------------------------------------------------
            // DRIVE ROBOT
            if (gamepad1.x) {
                multiplier = 0.5;
            }
            if (gamepad1.y) {
                multiplier = 0.3;
            }
            if (gamepad1.a) {
                multiplier = 0.8;
            }

            if(gamepad1.dpad_down) {
                fldrive.setPower(-0.8);
                frdrive.setPower(-0.8);
                bldrive.setPower(-0.8);
                brdrive.setPower(-0.8);
            }
            if(gamepad1.dpad_up) {
                fldrive.setPower(0.8);
                frdrive.setPower(0.8);
                bldrive.setPower(0.8);
                brdrive.setPower(0.8);
            }
            if(gamepad1.dpad_left) {
                fldrive.setPower(0.8);
                frdrive.setPower(-0.8);
                bldrive.setPower(-0.8);
                brdrive.setPower(0.8);
            }
            if(gamepad1.dpad_right) {
                fldrive.setPower(-0.8);
                frdrive.setPower(0.8);
                bldrive.setPower(0.8);
                brdrive.setPower(-0.8);
            }

            mecanum(fldrive, frdrive, bldrive, brdrive, multiplier);
            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT

            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT

            if (gamepad2.a) {
                rgrab.setPower(-0.32);
                lgrab.setPower(-0.8);
                stopped = false;
            }
            if (gamepad2.b) {
                rgrab.setPower(-0.7);
                lgrab.setPower(-0.7);
                stopped = false;
            }
            if (gamepad2.x) {
                rgrab.setPower(0.0);
                rgrab.setPower(0.0);
                stopped = true;
            }
            if (gamepad2.y) {
                sleep(250);
                direction = -direction;
            }

            if (stopped) {
                rgrab.setPower(-direction * 0.7 * Math.signum(gamepad2.right_trigger));
                lgrab.setPower(-direction * 0.7 * Math.signum(gamepad2.left_trigger));
            }
            lift.setPower(gamepad2.right_stick_y);
            if (gamepad2.dpad_up) {
                flipper.setPosition(1.0);
                sleep(500);
                flipper.setPosition(0.0);
            }
            if (gamepad2.dpad_right) {
                sleep(250);
                flipper.setPosition(1.0);
            }
            if (gamepad2.dpad_left) {
                sleep(250);
                flipper.setPosition(0.0);
            }
            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // TELEMETRY
            telemetry.addData("multiplier", multiplier);
            telemetry.update();
            //-----------------------------------------------------------------------------
        }
    }

    public void mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, double multiplier) {
        double joyly = gamepad1.left_stick_y;
        double joylx = -(gamepad1.left_stick_x);
        double joyrx = -(gamepad1.right_stick_x);
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joyly, joylx);
        double v0 = joyrx;
        //v0 = bumpers;
        double v1 = vd*Math.sin(theta+(Math.PI/4))-v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))-v0; //fr
        double v3 = vd*Math.cos(theta+(Math.PI/4))+v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))+v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));

        if (max > 1.0) {
            fl.setPower(multiplier * (v1/max));
            fr.setPower(multiplier * (v2/max));
            bl.setPower(multiplier * (v3/max));
            br.setPower(multiplier * (v4/max));
        }
        else if (max != 0) {
            fl.setPower(multiplier * (v1));
            fr.setPower(multiplier * (v2));
            bl.setPower(multiplier * (v3));
            br.setPower(multiplier * (v4));
        }
        else {
            fl.setPower(0.0);
            fr.setPower(0.0);
            bl.setPower(0.0);
            br.setPower(0.0);
        }
    }

}
