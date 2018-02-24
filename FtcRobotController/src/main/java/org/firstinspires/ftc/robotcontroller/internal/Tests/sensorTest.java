package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sahith on 10/29/17.
 */
public class sensorTest extends LinearOpMode {

    ColorSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.colorSensor.get("jewelSensor");
        String move = "NONE";
        waitForStart();
        while(opModeIsActive()) {
            if (sensor.red() > sensor.blue()) {
                move = "RIGHT";
            } else if (sensor.blue() > sensor.red()) {
                move = "LEFT";
            } else {
                move = "NONE";
            }
            telemetry.addData("move", move);
            telemetry.addData("Red value", sensor.red());
            telemetry.addData("Blue value", sensor.blue());
            telemetry.update();
        }
    }

}
