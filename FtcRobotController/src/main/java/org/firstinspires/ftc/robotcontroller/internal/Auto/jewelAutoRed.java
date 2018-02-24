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

    final static double CAT_STOW = 0.85944444444444444444;
    final static double CAT_EXTEND = 0.3094444444444444444;
    final static double KNOCK_CENTER = 0.23944444444444444444452;
    final static double KNOCK_RIGHT = .75;
    final static double KNOCK_LEFT = .0;
    final static double KNOCK_STOW = .359444444444444444444445;

    @Override
    public void runOpMode() throws InterruptedException {
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        waitForStart();

        jewelAuto("RED");
    }

    public void jewelAuto(String team) {
        extend();
        sleep(2000);
        if (team.equals("RED")) {
            doJewel(jewelSensor.red(), jewelSensor.blue());
        }
        else if (team.equals("BLUE")){
            doJewel(jewelSensor.blue(), jewelSensor.red());
        }
    }

    public void doJewel(int color1, int color2) {
        if (color1 > color2) {
            right();
            knock.setPosition(KNOCK_CENTER);
            sleep(250);
            stow();
        }
        else {
            left();
            knock.setPosition(KNOCK_CENTER);
            sleep(250);
            stow();
        }
    }

    public void stow() {
        cat.setPosition(CAT_EXTEND + 0.3);
        sleep(250);
        knock.setPosition(KNOCK_STOW);
        sleep(250);
        cat.setPosition(CAT_STOW);
        sleep(250);
    }

    public void extend() {
        cat.setPosition(CAT_EXTEND + 0.3);
        sleep(250);
        knock.setPosition(KNOCK_CENTER);
        sleep(250);
        cat.setPosition(CAT_EXTEND);
        sleep(250);
    }

    public void right() {
        knock.setPosition(KNOCK_RIGHT);
        sleep(250);
    }

    public void left() {
        knock.setPosition(KNOCK_LEFT);
        sleep(250);
    }
}
