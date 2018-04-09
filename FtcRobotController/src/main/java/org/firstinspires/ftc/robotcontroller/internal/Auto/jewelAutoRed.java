package org.firstinspires.ftc.robotcontroller.internal.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 2/18/18.
 */

public class jewelAutoRed extends LinearOpMode {
    Servo cat, knock;
    ColorSensor jewelSensor;

    final static double CAT_STOW = 0.619444444444444444445;
    final static double CAT_EXTEND = 0.1694444444444444445;
    final static double KNOCK_RIGHT = .2400000000000000005; //0.56
    final static double KNOCK_LEFT = .85944444444444444445;  //0.18
    final static double KNOCK_CENTER = 0.60000000000000001;

    @Override
    public void runOpMode() throws InterruptedException {
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        cat.setPosition(CAT_STOW);
        knock.setPosition(KNOCK_CENTER);

        waitForStart();

        cat.setPosition(CAT_EXTEND);
        sleep(1000);
        if(jewelSensor.red() > jewelSensor.blue()) {
            knock.setPosition(KNOCK_LEFT);
            sleep(1000);
        }
        else if(jewelSensor.blue() > jewelSensor.red()) {
            knock.setPosition(KNOCK_RIGHT);
            sleep(1000);
        }

    }
}
