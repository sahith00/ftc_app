package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by sahith on 2/5/18.
 */

public class Intake {
    public DcMotor rintake, lintake;

    public Intake(HardwareMap hardwareMap) {
        this.rintake = hardwareMap.dcMotor.get("rintake");
        this.lintake = hardwareMap.dcMotor.get("lintake");
    }

    public void runIntake(double rpower, double lpower) {
        rintake.setPower(rpower);
        lintake.setPower(lpower);
    }
}
