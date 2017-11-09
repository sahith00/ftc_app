package org.firstinspires.ftc.robotcontroller.internal.Google;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 10/9/17.
 */
public class googleTeleOp extends LinearOpMode{
    DcMotor frdrive, fldrive, brdrive, bldrive;
    //DcMotor lift, reliclift;
    Servo cat, knock;
    Servo lgrab, rgrab;
    Servo relicgrab;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fl");
        frdrive = hardwareMap.dcMotor.get("fr");
        bldrive = hardwareMap.dcMotor.get("bl");
        brdrive = hardwareMap.dcMotor.get("br");
        //lift = hardwareMap.dcMotor.get("lift");
        //reliclift = hardwareMap.dcMotor.get("reliclift");
        lgrab = hardwareMap.servo.get("lgrab");
        rgrab = hardwareMap.servo.get("rgrab");
        relicgrab = hardwareMap.servo.get("relicgrab");
        boolean grab = false;
        boolean grabrelic = false;
        lgrab.setPosition(0);
        rgrab.setPosition(0.9);
        relicgrab.setPosition(0);
        //lift.setPower(0);

        int count = 0;
        double multipliers[] = new double[3];
        multipliers[0] = 1.0;
        multipliers[1] = 0.8;
        multipliers[2] = 0.4;

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.x) {
                sleep(250);
                count++;
                if (count > 2) {
                    count = 0;
                }
            }
            mecanum(fldrive, frdrive, bldrive, brdrive, multipliers[count]);

            //lift.setPower(gamepad2.right_stick_y);
            //reliclift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

            if (gamepad2.a) {
                grab = true;
            }
            if (gamepad2.b) {
                lgrab.setPosition(0.35);
                rgrab.setPosition(0.55);
                grab = false;
            }
            if (gamepad2.y) {
                lgrab.setPosition(0.0);
                rgrab.setPosition(0.9);
            }
            if (grab) {
                lgrab.setPosition(0.45);
                rgrab.setPosition(0.45);
            }

            if (gamepad2.x) {
                sleep(250);
                grabrelic = !grabrelic;
            }
            if(grabrelic) {
                relicgrab.setPosition(0.5);
            }
            if(!grabrelic) {
                relicgrab.setPosition(0);
            }

            telemetry.addData("Position", lgrab.getPosition());
            telemetry.addData("RPosition", rgrab.getPosition());
            telemetry.addData("Grab", grab);
            telemetry.update();
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
