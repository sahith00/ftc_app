package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 10/12/17.
 */
public class servoTest extends LinearOpMode {
    Servo servo1, servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.servo.get("lflip");
        servo2 = hardwareMap.servo.get("rflip");
        servo1.setPosition(0.5);
        servo2.setPosition(0.2395);
        double lchange = 0.01, rchange = 0.01;
        double cchange[] = new double[2];
        cchange[0] = 0.01;
        cchange[1] = 0.1;
        int count = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                sleep(250);
                lchange += cchange[count];
                if (lchange >= 1.0) {
                    lchange = 0.01;
                }
            }

            if (gamepad1.y) {
                sleep(250);
                rchange += cchange[count];
                if (rchange >= 1.0) {
                    rchange = 0.01;
                }
            }

            if (gamepad1.right_bumper) {
                sleep(500);
                count++;
                if (count == 2) {
                    count = 0;
                }
            }

            if (gamepad1.dpad_down) {
                sleep(500);
                double ltemp = servo1.getPosition() + lchange;
                if (ltemp >= 1.0) {
                    ltemp = 1.0;
                }
                servo1.setPosition(ltemp);
            }
            if (gamepad1.dpad_up) {
                sleep(500);
                double ltemp = servo1.getPosition() - lchange;
                if (ltemp < 0.0) {
                    ltemp = 0.0;
                }
                servo1.setPosition(ltemp);
            }

            if (gamepad1.a) {
                sleep(500);
                double rtemp = servo2.getPosition() - rchange;
                if (rtemp <= 0.0) {
                    rtemp = 0.0;
                }
                servo2.setPosition(rtemp);
            }
            if (gamepad1.b) {
                sleep(500);
                double rtemp = servo2.getPosition() + rchange;
                if (rtemp >= 1.0) {
                    rtemp = 1.0;
                }
                servo2.setPosition(rtemp);
            }

            telemetry.addData("cchange", cchange[count]);
            telemetry.addData("lchange", lchange);
            telemetry.addData("rchange", rchange);
            telemetry.addData("servo1", servo1.getPosition());
            telemetry.addData("servo2", servo2.getPosition());
            telemetry.update();
        }
    }
}