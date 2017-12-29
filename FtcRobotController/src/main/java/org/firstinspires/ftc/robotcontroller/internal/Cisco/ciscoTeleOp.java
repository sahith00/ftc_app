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
    DcMotor lift;
    Servo cat, knock;

    final static double CAT_STOW = 0.66;
    final static double KNOCK_CENTER = 0.38;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        lift = hardwareMap.dcMotor.get("lift");
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");

        fldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        lift.setPower(0);
        cat.setPosition(CAT_STOW);
        knock.setPosition(KNOCK_CENTER);

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

            //-----------------------------------------------------------------------------


            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            lift.setPower(Range.clip(gamepad2.right_stick_y + 0.5, -0.5, 1));
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
