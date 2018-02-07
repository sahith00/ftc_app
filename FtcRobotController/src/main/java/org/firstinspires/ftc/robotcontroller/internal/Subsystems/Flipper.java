package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 2/5/18.
 */

public class Flipper {
    public Servo rflip, lflip, stopper;

    final static double STOPPER_STOP = 0.959444444444444445;
    final static double STOPPER_DEPOSIT = 0.62000000000000001;
    final static double STOPPER_ZERO = 0.16944444444444452;
    final static double RFLIP_DEPOSIT = 0.789444444444444446;
    final static double RFLIP_ZERO = 0.18;
    final static double RFLIP_GRAB = 0.06;
    final static double LFLIP_DEPOSIT = 0.0294444444444444444;
    final static double LFLIP_ZERO = 0.619444444444444444445;
    final static double LFLIP_GRAB = 0.739444444444444444446;

    public Flipper(HardwareMap hardwareMap) {
        this.rflip = hardwareMap.servo.get("rflip");
        this.lflip = hardwareMap.servo.get("lflip");
        this.stopper = hardwareMap.servo.get("stopper");
    }

    public void grab() {
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
        stopper.setPosition(STOPPER_STOP);
    }
    public void zero() {
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
        stopper.setPosition(STOPPER_ZERO);
    }
    public void deposit() {
        rflip.setPosition(RFLIP_DEPOSIT);
        lflip.setPosition(LFLIP_DEPOSIT);
        stopper.setPosition(STOPPER_DEPOSIT);
    }
}
