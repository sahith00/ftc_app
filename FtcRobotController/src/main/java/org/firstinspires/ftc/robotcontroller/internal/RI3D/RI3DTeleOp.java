package org.firstinspires.ftc.robotcontroller.internal.RI3D;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by sahith on 9/9/17.
 */
public class RI3DTeleOp extends LinearOpMode{
    DcMotor fldrive, frdrive, bldrive, brdrive;
    DcMotor grabber;
    DcMotor lift1, lift2;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        grabber = hardwareMap.dcMotor.get("grabber");
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int threshold = 3;
        int count = 0;
        double multipliers[] = new double[3];
        multipliers[0] = 1.0;
        multipliers[1] = 0.8;
        multipliers[2] = 0.4;
        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.x) {
                sleep(500);
                count++;
                if (count > 2) {
                    count = 0;
                }
            }
            mecanum(fldrive, frdrive, bldrive, brdrive, multipliers[count]);

            if(gamepad1.a){
                grabber.setPower(0.5 - ((grabber.getCurrentPosition() / 550) * .5));
                if(grabber.getCurrentPosition() < (438 + threshold) && grabber.getCurrentPosition() > (416 - threshold)) {
                    grabber.setTargetPosition(grabber.getCurrentPosition());
                }
                else {
                    grabber.setTargetPosition(430);

                }
            }
            if (gamepad1.b) {
                grabber.setPower(0.5 - (((430 - grabber.getCurrentPosition()) / 550) * .5));
                grabber.setTargetPosition(0);
                sleep(500);
                grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.left_bumper) {
                sleep(1000);
                threshold++;
            }
            if (gamepad1.right_bumper) {
                sleep(1000);
                threshold--;
            }

            operateLift(gamepad2.right_stick_y);

            telemetry.addData("Encoder Count", grabber.getCurrentPosition());
            telemetry.addData("Threshold", threshold);
            telemetry.addData("Drive Multiplier ", multipliers[count]);
            telemetry.update();
        }
    }

    public void mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, double multiplier) {
        double joyly = -(gamepad1.left_stick_y);
        double joylx = gamepad1.left_stick_x;
        double joyrx = gamepad1.right_stick_x;
//        double bumpers = gamepad1.right_trigger-gamepad1.left_trigger;
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joylx, joyly);
        double v0 = joyrx;
        //v0 = bumpers;
        double v1 = vd*Math.sin(theta+(Math.PI/4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))-v0; //fr
        double v3 = vd*Math.cos(theta+(Math.PI/4))+v0; //bl
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

    public void operateLift(double power) {
        if (power > 0.2) {
            lift1.setPower(0.8);
            lift2.setPower(0.8);
        }
        else if (power < -0.2) {
            lift1.setPower(-0.8);
            lift2.setPower(-0.8);
        }
        else {
            lift1.setPower(0.0);
            lift2.setPower(0.0);
        }
    }
}
