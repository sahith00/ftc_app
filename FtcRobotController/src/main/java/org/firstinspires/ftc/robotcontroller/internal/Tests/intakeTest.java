package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by vulcanrobotics8375 on 1/4/18.
 */

public class intakeTest extends LinearOpMode {
    DcMotor lintake, rintake;
    double lintakep, rintakep;
    public void runOpMode() {
        lintake = hardwareMap.dcMotor.get("lintake");
        rintake = hardwareMap.dcMotor.get("rintake");
        lintakep = -1.0;
        rintakep = -1.0;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                sleep(250);
                lintakep += .1;
                if (lintakep > 1.0) {
                    lintakep = 1.0;
                }
            }
            if (gamepad1.x) {
                sleep(250);
                lintakep -= .1;
                if (lintakep < -1.0) {
                    lintakep = -1.0;
                }
            }
            if (gamepad1.b) {
                sleep(250);
                rintakep += .1;
                if (rintakep > 1.0) {
                    rintakep = 1.0;
                }
            }
            if (gamepad1.a) {
                sleep(250);
                rintakep -= .1;
                if (rintakep < -1.0) {
                    rintakep = -1.0;
                }
            }
            if (gamepad1.right_bumper) {
                lintakep = .0;
                rintakep = .0;
            }
            lintake.setPower(lintakep);
            rintake.setPower(rintakep);
            telemetry.addData("Controls: ", "Y increases rintakep, X decreases; B increases lintakep, A decreases; Right bumper sets both to 0");
            telemetry.addData("lintakep: ", lintake.getPower());
            telemetry.addData("rintakep: ", rintake.getPower());
            telemetry.update();
        }
    }
}