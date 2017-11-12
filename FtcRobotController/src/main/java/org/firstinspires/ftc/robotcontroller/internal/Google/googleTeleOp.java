package org.firstinspires.ftc.robotcontroller.internal.Google;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sahith on 10/9/17.
 */
public class googleTeleOp extends LinearOpMode{
    DcMotor frdrive, fldrive, brdrive, bldrive;
    //DcMotor lift, reliclift;
    Servo cat, knock;
    Servo relicgrab;
    CRServo lgrab, rgrab, lbrush, rbrush, grablift;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        //lift = hardwareMap.dcMotor.get("lift");
        //reliclift = hardwareMap.dcMotor.get("reliclift");
        relicgrab = hardwareMap.servo.get("relicgrab");
        lgrab = hardwareMap.crservo.get("lservo");
        rgrab = hardwareMap.crservo.get("rservo");
        lbrush = hardwareMap.crservo.get("lbrush");
        rbrush = hardwareMap.crservo.get("rbrush");
        grablift = hardwareMap.crservo.get("grablift");

        boolean forward = true, reverse = false;
        boolean grabrelic = false;

        fldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        //lift.setPower(0);
        //reliclift.setPower(0);
        relicgrab.setPosition(0);
        lgrab.setPower(-0.08);
        rgrab.setPower(0);
        lbrush.setPower(0.05);
        rbrush.setPower(0);
        grablift.setPower(0);


        int count = 0;
        double multipliers[] = new double[3];
        multipliers[0] = 1.0;
        multipliers[1] = 0.8;
        multipliers[2] = 0.4;

        waitForStart();

        while(opModeIsActive()) {

            //-----------------------------------------------------------------------------
            // DRIVE ROBOT
            if (gamepad1.x) {
                sleep(250);
                count++;
                if (count > 2) {
                    count = 0;
                }
            }
            if (gamepad1.y) {
                sleep(250);
                count--;
                if (count < 0) {
                    count = 2;
                }
            }
            mecanum(fldrive, frdrive, bldrive, brdrive, multipliers[count]);
            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT
            //reliclift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            if (gamepad1.a) {
                sleep(250);
                grabrelic = !grabrelic;
            }
            if(grabrelic) {
                relicgrab.setPosition(0.5);
            }
            if(!grabrelic) {
                relicgrab.setPosition(0);
            }
            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            if (gamepad2.x) {
                forward = true;
                reverse = false;
            }
            if (gamepad2.y) {
                reverse = true;
                forward = false;
            }
            if(gamepad2.a) {
                rgrab.setPower(0);
                rbrush.setPower(0);
            }
            if(gamepad2.b) {
                lgrab.setPower(-0.08);
                lbrush.setPower(0.05);
            }
            if(forward) {
                rgrab.setPower((double)(gamepad2.right_trigger));
                rbrush.setPower((double)(gamepad2.right_trigger));
                lgrab.setPower(Range.clip((double) (-gamepad2.left_trigger), -1, -0.08));
                lbrush.setPower((double) (-gamepad2.left_trigger) + 0.05);
            }
            if(reverse) {
                rgrab.setPower((double)(-gamepad2.right_trigger));
                rbrush.setPower((double)(-gamepad2.right_trigger));
                lgrab.setPower((double)(gamepad2.left_trigger)-0.08);
                lbrush.setPower(Range.clip((double) (gamepad2.left_trigger), 0.05, 1));
            }
            grablift.setPower(-gamepad2.right_stick_y);
            //lift.setPower(gamepad2.left_stick_y);
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
//        double bumpers = gamepad1.right_trigger-gamepad1.left_trigger;
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
