package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 10/12/17.
 */
public class servoTest extends LinearOpMode{
    Servo lgrab, rgrab;

    @Override
    public void runOpMode() throws InterruptedException {
        lgrab = hardwareMap.servo.get("lgrab");
        rgrab = hardwareMap.servo.get("rgrab");
        lgrab.setPosition(0);
        rgrab.setPosition(1);
        double lchange = 0.1, rchange = 0.1;

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                sleep(500);
                lchange += 0.1;
                if (lchange >= 1.0) {
                    lchange = 0.1;
                }
            }

            if(gamepad1.y) {
                sleep(500);
                rchange += 0.1;
                if (rchange >= 1.0) {
                    rchange = 0.1;
                }
            }

            if(gamepad1.right_bumper) {
                sleep(500);
                lchange = 0.1;
                rchange = 0.1;
            }

            if (gamepad1.dpad_down) {
                sleep(500);
                double ltemp = lgrab.getPosition() + lchange;
                if (ltemp >= 1.0) {
                    ltemp = 1.0;
                }
                lgrab.setPosition(ltemp);
            }
            if (gamepad1.dpad_up) {
                sleep(500);
                double ltemp = lgrab.getPosition() - lchange;
                if (ltemp <= 0.0) {
                    ltemp = 0.0;
                }
                lgrab.setPosition(ltemp);
            }

            if (gamepad1.a) {
                sleep(500);
                double rtemp = rgrab.getPosition() - rchange;
                if (rtemp <= 0.0) {
                    rtemp = 0.0;
                }
                rgrab.setPosition(rtemp);
            }
            if (gamepad1.b) {
                sleep(500);
                double rtemp = rgrab.getPosition() + rchange;
                if (rtemp >= 1.0) {
                    rtemp = 1.0;
                }
                rgrab.setPosition(rtemp);
            }

            telemetry.addData("lchange", lchange);
            telemetry.addData("rchange", rchange);
            telemetry.addData("lgrab", lgrab.getPosition());
            telemetry.addData("rgrab", rgrab.getPosition());
            telemetry.update();
        }
    }
}
