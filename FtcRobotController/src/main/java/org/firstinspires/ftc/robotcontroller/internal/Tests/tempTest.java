package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 11/8/17.
 */
public class tempTest extends LinearOpMode {
    Servo cat, knock;

    @Override
    public void runOpMode() throws InterruptedException {
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");

        cat.setPosition(0.345);
        waitForStart();

        cat.setPosition(0.345);
        sleep(1000);
        cat.setPosition(0.58);
        while(cat.getPosition() != 0.58) {

        }
        cat.setPosition(0.345);
        while(cat.getPosition() != 0.345) {

        }
        cat.setPosition(0.58);
        while(cat.getPosition() != 0.58) {

        }
        cat.setPosition(0.345);
        while(cat.getPosition() != 0.345) {

        }
        cat.setPosition(0.58);
        sleep(1000);

        telemetry.update();
    }
}
