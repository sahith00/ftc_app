package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sahith on 11/11/17.
 */
public class grabberTest extends LinearOpMode{
    CRServo lservo, rservo;

    @Override
    public void runOpMode() throws InterruptedException {
        lservo = hardwareMap.crservo.get("lservo");
        rservo = hardwareMap.crservo.get("rservo");

        lservo.setPower(-0.08);
        rservo.setPower(0);

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
            }
            if(gamepad1.b) {
                lservo.setPower(-0.08);
            }
            if(forward) {
                rservo.setPower((double)(gamepad1.right_trigger));
                lservo.setPower(Range.clip((double)(-gamepad1.left_trigger), -1, -0.08));
            }
            if(reverse) {
                rservo.setPower((double)(-gamepad1.right_trigger));
                lservo.setPower((double)(gamepad1.left_trigger)-0.08);
            }
            telemetry.update();
        }
    }
}
