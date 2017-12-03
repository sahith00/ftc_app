package org.firstinspires.ftc.robotcontroller.internal.Google;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sahith on 10/9/17.
 */
public class googleTeleOp extends LinearOpMode{
    DcMotor frdrive, fldrive, brdrive, bldrive;
    DcMotor lift1, lift2, grablift, reliclift;
    Servo body, arm;
    Servo cat, knock;
    CRServo lgrab, rgrab, lbrush, rbrush;

    final static double GRAB_BODY = 0.5;
    final static double DROP_BODY = 0.0;
    final static double STOW_BODY = 1.0;
    final static double EXTEND_ARM = 0.0;
    final static double GRAB_ARM = 0.54;
    final static double DROP_ARM = 0.3;
    final static double STOW_ARM = 0.63;
    final static double CAT_STOW = 0.66;
    final static double KNOCK_CENTER = 0.38;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        grablift = hardwareMap.dcMotor.get("grablift");
        reliclift = hardwareMap.dcMotor.get("reliclift");
        body = hardwareMap.servo.get("body");
        arm = hardwareMap.servo.get("arm");
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");
        lgrab = hardwareMap.crservo.get("lgrab");
        rgrab = hardwareMap.crservo.get("rgrab");
        lbrush = hardwareMap.crservo.get("lbrush");
        rbrush = hardwareMap.crservo.get("rbrush");

        boolean forward = true, reverse = false;
        boolean extendarm = false, grabrelic = false, droprelic = false, stowrelic = true;

        fldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grablift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reliclift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        lift1.setPower(0);
        lift2.setPower(0);
        grablift.setPower(0);
        reliclift.setPower(0);
        body.setPosition(STOW_BODY);
        arm.setPosition(STOW_ARM);
        cat.setPosition(CAT_STOW);
        knock.setPosition(KNOCK_CENTER);
        lgrab.setPower(0);
        rgrab.setPower(0);
        lbrush.setPower(0.05);
        rbrush.setPower(0);


        double multiplier = 1.0;

        waitForStart();

        while(opModeIsActive()) {
            cat.setPosition(CAT_STOW);
            knock.setPosition(KNOCK_CENTER);

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

            mecanum(fldrive, frdrive, bldrive, brdrive, multiplier);
            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT
            if (gamepad2.dpad_right) {
                reliclift.setPower(-0.4);
            }
            if (gamepad2.dpad_left) {
                reliclift.setPower(0.4);
            }
            if (gamepad2.dpad_up) {
                reliclift.setPower(0);
            }

            if (gamepad2.a) {
                extendarm = false;
                grabrelic = true;
                droprelic = false;
                stowrelic = false;
            }
            if (gamepad2.b) {
                extendarm = false;
                grabrelic = false;
                droprelic = false;
                stowrelic = true;
            }
            if (gamepad2.x) {
                extendarm = false;
                grabrelic = false;
                droprelic = true;
                stowrelic = false;
            }
            if (gamepad2.y) {
                extendarm = true;
                grabrelic = false;
                droprelic = false;
                stowrelic = false;
            }
            if(extendarm) {
                body.setPosition(GRAB_BODY);
                arm.setPosition(EXTEND_ARM);
            }
            if (grabrelic) {
                body.setPosition(GRAB_BODY);
                arm.setPosition(GRAB_ARM);
            }
            else if (droprelic){
                body.setPosition(DROP_BODY);
                arm.setPosition(DROP_ARM);
            }
            else if (stowrelic) {
                body.setPosition(STOW_BODY);
                arm.setPosition(STOW_ARM);
            }
            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            if (gamepad2.right_bumper) {
                forward = true;
                reverse = false;
            }
            if (gamepad2.left_bumper) {
                reverse = true;
                forward = false;
            }

            if (forward) {
                rgrab.setPower((double)(gamepad2.right_trigger));
                rbrush.setPower((double)(gamepad2.right_trigger));
                lgrab.setPower((double) (-gamepad2.left_trigger));
                lbrush.setPower((double) (-gamepad2.left_trigger) + 0.05);
            }
            if(reverse) {
                rgrab.setPower((double)(-gamepad2.right_trigger));
                rbrush.setPower((double)(-gamepad2.right_trigger));
                lgrab.setPower((double)(gamepad2.left_trigger));
                lbrush.setPower(Range.clip((double) (gamepad2.left_trigger), 0.05, 1));
            }

            grablift.setPower(gamepad2.right_stick_y);
            lift1.setPower(Range.clip(Math.signum(gamepad2.left_stick_y), -0.9, 0.6));
            lift2.setPower(Range.clip(Math.signum(gamepad2.left_stick_y), -0.9, 0.6));
            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // TELEMETRY

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
        double v1 = vd*Math.sin(theta+(Math.PI/4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))+v0; //fr
        double v3 = vd*Math.cos(theta+(Math.PI/4))-v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));

        if (max != 0) {
            fl.setPower(multiplier * (v1/max));
            fr.setPower(multiplier * (v2/max));
            bl.setPower(multiplier * (v3/max));
            br.setPower(multiplier * (v4/max));
        } else {
            fl.setPower(0.0);
            fr.setPower(0.0);
            bl.setPower(0.0);
            br.setPower(0.0);
        }
    }

}