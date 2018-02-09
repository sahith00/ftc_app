package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/**
 * Created by sahith on 2/5/18.
 */

public class Jewel {
    public Servo cat, knock;
    public ColorSensor jewelSensor;

    final static double CAT_STOW = 0.85944444444444444444;
    final static double CAT_EXTEND = 0.3094444444444444444;
    final static double KNOCK_CENTER = 0.23944444444444444444452;
    final static double KNOCK_RIGHT = .75;
    final static double KNOCK_LEFT = .0;
    final static double KNOCK_STOW = .359444444444444444444445;

    public Jewel(HardwareMap hardwareMap) {
        this.cat = hardwareMap.servo.get("cat");
        this.knock = hardwareMap.servo.get("knock");
        this.jewelSensor = hardwareMap.colorSensor.get("jewelSensor");
    }

    public void stowCat() {
        cat.setPosition(CAT_STOW);
    }
    public void halfCat() {
        cat.setPosition(CAT_EXTEND + 0.3);
    }
    public void extendCat() {
        cat.setPosition(CAT_EXTEND);
    }

    public void knockRight() {
        knock.setPosition(KNOCK_RIGHT);
    }
    public void knockLeft() {
        knock.setPosition(KNOCK_LEFT);
    }
    public void knockCenter() {
        knock.setPosition(KNOCK_CENTER);
    }
    public void stowKnock() {
        knock.setPosition(KNOCK_STOW);
    }

    public boolean isBlue() {
        return (jewelSensor.blue() > jewelSensor.red());
    }

    public boolean isRed() {
        return (jewelSensor.red() > jewelSensor.blue());
    }
}
