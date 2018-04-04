package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.hardware.Sensor;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by sahith on 1/20/18.
 */

public class jewelAutoTest extends LinearOpMode {
    Servo cat, knock;
    ColorSensor jewelSensor;

    final static double CAT_STOW = 0.79;
    final static double CAT_EXTEND = 0.31;
    final static double KNOCK_CENTER = 0.37;
    final static double KNOCK_RIGHT = .66; //0.56
    final static double KNOCK_LEFT = .08;  //0.18
    final static double KNOCK_STOW = .42;

    String team = "RED";

    boolean jewelStow = true;

    @Override
    public void runOpMode() throws InterruptedException {
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        cat.setPosition(CAT_STOW);
        knock.setPosition(KNOCK_STOW);

        while (!isStarted()) {
            if (gamepad1.x) {
                team = "BLUE";
            }
            if (gamepad1.b) {
                team = "RED";
            }
            telemetry.update();
            idle();
        }

        sleep(500);
        jewelAuto(team);
    }

    public void jewelAuto(String team) {
        extend();
        sleep(1000);
        if (team.equals("RED")) {
            if (jewelSensor.blue() > jewelSensor.red()) {
                telemetry.addData("Blue", jewelSensor.blue());
                telemetry.addData("Red", jewelSensor.red());
                telemetry.update();
                left();
                center();
                stow();
            }
            else if (jewelSensor.red() > jewelSensor.blue()) {
                telemetry.addData("Blue", jewelSensor.blue());
                telemetry.addData("Red", jewelSensor.red());
                telemetry.update();
                right();
                center();
                jewelStow = false;
            }
            else {
                stow();
            }
        }
        else if (team.equals("BLUE")){
            if (jewelSensor.red() > jewelSensor.blue()) {
                telemetry.addData("Blue", jewelSensor.blue());
                telemetry.addData("Red", jewelSensor.red());
                telemetry.update();
                left();
                center();
                stow();
            }
            else if (jewelSensor.blue() > jewelSensor.red()) {
                telemetry.addData("Blue", jewelSensor.blue());
                telemetry.addData("Red", jewelSensor.red());
                telemetry.update();
                right();
                center();
                jewelStow = false;
            }
            else {
                stow();
            }
        }
    }

    public void stow() {
        cat.setPosition(CAT_EXTEND + 0.3);
        sleep(200);
        knock.setPosition(KNOCK_STOW);
        sleep(200);
        cat.setPosition(CAT_STOW);
        sleep(200);
        jewelStow = true;
    }

    public void extend() {
        cat.setPosition(CAT_EXTEND + 0.3);
        sleep(200);
        knock.setPosition(KNOCK_CENTER);
        sleep(200);
        cat.setPosition(CAT_EXTEND);
        sleep(200);
    }

    public void right() {
        knock.setPosition(KNOCK_RIGHT);
        sleep(200);
    }

    public void left() {
        knock.setPosition(KNOCK_LEFT);
        sleep(200);
    }

    public void center() {
        knock.setPosition(KNOCK_CENTER);
        sleep(200);
    }
}
