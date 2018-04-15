package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by sahith on 4/14/18.
 */

public class distanceSensorTest extends LinearOpMode {

    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}