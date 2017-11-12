package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sahith on 11/11/17.
 */
public class grabberTest extends LinearOpMode{
    CRServo lservo, rservo, lbrush, rbrush, glift;

    @Override
    public void runOpMode() throws InterruptedException {
        lservo = hardwareMap.crservo.get("lservo");
        rservo = hardwareMap.crservo.get("rservo");
        lbrush = hardwareMap.crservo.get("lbrush");
        rbrush = hardwareMap.crservo.get("rbrush");
        glift = hardwareMap.crservo.get("glift");

        lservo.setPower(-0.08);
        rservo.setPower(0);
        lbrush.setPower(0.05);
        rbrush.setPower(0);
        glift.setPower(0);

        boolean forward = false, reverse = false;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                forward = true;
                reverse = false;
            }
            if (gamepad1.y) {
                reverse = true;
                forward = false;
            }
            if(gamepad1.a) {
                rservo.setPower(0);
                rbrush.setPower(0);
            }
            if(gamepad1.b) {
                lservo.setPower(-0.08);
                lbrush.setPower(0.05);
            }
            if(forward) {
                rservo.setPower((double)(gamepad1.right_trigger));
                rbrush.setPower((double)(gamepad1.right_trigger));
                lservo.setPower(Range.clip((double)(-gamepad1.left_trigger), -1, -0.08));
                lbrush.setPower((double)(-gamepad1.left_trigger)+0.05);
            }
            if(reverse) {
                rservo.setPower((double)(-gamepad1.right_trigger));
                rbrush.setPower((double)(-gamepad1.right_trigger));
                lservo.setPower((double)(gamepad1.left_trigger)-0.08);
                lbrush.setPower(Range.clip((double)(gamepad1.left_trigger), 0.05, 1));
            }
            glift.setPower(-gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
