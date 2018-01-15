package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by sahith on 1/13/18.
 */

public class lineFollowingTest3 extends LinearOpMode {
    ColorSensor lineSensor1, lineSensor2, lineSensor3;
    DcMotor fr, fl, br, bl;
    int desiredValue = 23;

    @Override
    public void runOpMode() throws InterruptedException {
        lineSensor1 = hardwareMap.colorSensor.get("lineSensor1");
        lineSensor2 = hardwareMap.colorSensor.get("lineSensor2");
        lineSensor3 = hardwareMap.colorSensor.get("lineSensor3");
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);

        while(!isStarted()) {
            telemetry.update();
            idle();
        }

        while(opModeIsActive()) {
            setBack(0.0);
            if(lineSensor1.blue() > lineSensor2.blue() && lineSensor1.blue() > desiredValue) {
                goLeft();
            }
            else if(lineSensor2.blue() > lineSensor1.blue() && lineSensor2.blue() > desiredValue) {
                goRight();
            }
            else if(lineSensor1.blue() < desiredValue && lineSensor2.blue() < desiredValue) {
                if (lineSensor3.blue() > desiredValue) {
                    goRight();
                }
                else if (lineSensor3.blue() < desiredValue) {
                    goLeft();
                }
            }
        }
    }

    public void setForward(double lpower) {
        fl.setPower(lpower);
        fr.setPower(lpower);
    }
    public void setBack(double rpower) {
        bl.setPower(rpower);
        br.setPower(rpower);
    }

    public void goRight() {
        fl.setPower(0.09);
        fr.setPower(0.05);
    }
    public void goLeft() {
        fl.setPower(0.05);
        fr.setPower(0.09);
    }
}
