package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 10/12/17.
 */
public class jewelServoTest extends LinearOpMode{
    Servo cat, knock;

    @Override
    public void runOpMode() throws InterruptedException {
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");
        cat.setPosition(0);
        knock.setPosition(1);
        double cchange = 0.1, kchange = 0.1;

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                sleep(500);
                cchange += 0.1;
                if (cchange >= 1.0) {
                    cchange = 0.1;
                }
            }

            if(gamepad1.y) {
                sleep(500);
                kchange += 0.1;
                if (kchange >= 1.0) {
                    kchange = 0.1;
                }
            }

            if(gamepad1.right_bumper) {
                sleep(500);
                cchange = 0.1;
                kchange = 0.1;
            }

            if (gamepad1.dpad_down) {
                sleep(500);
                double ctemp = cat.getPosition() + cchange;
                if (ctemp >= 1.0) {
                    ctemp = 1.0;
                }
                cat.setPosition(ctemp);
            }
            if (gamepad1.dpad_up) {
                sleep(500);
                double ctemp = cat.getPosition() - cchange;
                if (ctemp <= 0.0) {
                    ctemp = 0.0;
                }
                cat.setPosition(ctemp);
            }

            if (gamepad1.a) {
                sleep(500);
                double ktemp = knock.getPosition() - kchange;
                if (ktemp <= 0.0) {
                    ktemp = 0.0;
                }
                knock.setPosition(ktemp);
            }
            if (gamepad1.b) {
                sleep(500);
                double ktemp = knock.getPosition() + kchange;
                if (ktemp >= 1.0) {
                    ktemp = 1.0;
                }
                knock.setPosition(ktemp);
            }

            telemetry.addData("cchange", cchange);
            telemetry.addData("kchange", kchange);
            telemetry.addData("cat", cat.getPosition());
            telemetry.addData("knock", knock.getPosition());
            telemetry.update();
        }
    }
}
