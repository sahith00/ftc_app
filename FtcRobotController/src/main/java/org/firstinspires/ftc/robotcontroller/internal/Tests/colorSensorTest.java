package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by sahith on 12/24/17.
 */

public class colorSensorTest extends LinearOpMode {
    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("lineSensor");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Red Value", colorSensor.red());
            telemetry.addData("Blue Value", colorSensor.blue());
            telemetry.update();
        }
    }
}
