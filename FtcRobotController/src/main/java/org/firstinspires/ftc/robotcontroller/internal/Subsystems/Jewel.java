package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 2/5/18.
 */

public class Jewel {
    public Servo cat, knock;
    public ColorSensor jewelSensor;

    final static double CAT_STOW = 0.79;
    final static double CAT_EXTEND = 0.31;

    public Jewel(HardwareMap hardwareMap) {
        this.cat = hardwareMap.servo.get("cat");
        this.knock = hardwareMap.servo.get("knock");
        this.jewelSensor = hardwareMap.colorSensor.get("jewelSensor");
    }

    public void stowCat() {
        cat.setPosition(CAT_STOW);
    }
    public void extendCat() {
        cat.setPosition(CAT_EXTEND);
    }

    public boolean isBlue() {
        return (jewelSensor.blue() > jewelSensor.red());
    }

    public boolean isRed() {
        return (jewelSensor.red() > jewelSensor.blue());
    }
}
