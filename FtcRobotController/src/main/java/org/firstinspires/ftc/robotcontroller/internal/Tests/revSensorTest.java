package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;


/**
 * Created by sahith on 3/18/18.
 */

public class revSensorTest extends LinearOpMode {

    NormalizedColorSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "revSensor");
        while(opModeIsActive()) {
            telemetry.addData("blue", sensor.getNormalizedColors().blue);
            telemetry.addData("red", sensor.getNormalizedColors().red);
            telemetry.addData("green", sensor.getNormalizedColors().green);
            telemetry.addData("alpha", sensor.getNormalizedColors().alpha);
            telemetry.addData("color", sensor.getNormalizedColors().toColor());
            telemetry.update();
        }
    }
}
