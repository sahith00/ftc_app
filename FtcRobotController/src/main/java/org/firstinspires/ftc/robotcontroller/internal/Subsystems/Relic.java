package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 2/9/18.
 */

public class Relic {
    public DcMotor relicLift;
    public Servo arm, claw;

    public Relic(HardwareMap hardwareMap) {
        this.relicLift = hardwareMap.dcMotor.get("relicLift");
        this.arm = hardwareMap.servo.get("arm");
        this.claw = hardwareMap.servo.get("claw");
    }

    public void runRelic(double power) {
        relicLift.setPower(power);
    }

    public void openArm() {
        arm.setPosition(.5);
    }

    public void closeArm() {
        arm.setPosition(0);
    }

    public void stow() {
        arm.setPosition(0);
        claw.setPosition(0);
    }

    public void extendRelic() {
        claw.setPosition(0.5);
    }
}
