package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by sahith on 2/5/18.
 */

public class Lift {
    public DcMotor lift;

    public Lift(HardwareMap hardwareMap) {
        this.lift = hardwareMap.dcMotor.get("lift");
    }

    public int getLiftPosition() {
        return lift.getCurrentPosition();
    }

    public void runLift(double power, int position) {
        lift.setPower(power);
        lift.setTargetPosition(position);
    }
}
