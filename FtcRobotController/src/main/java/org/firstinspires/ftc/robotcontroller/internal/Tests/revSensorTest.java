package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Created by sahith on 3/18/18.
 */

public class revSensorTest extends LinearOpMode {

    DistanceSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DistanceSensor.class, "revSensor");
        while(opModeIsActive()) {
            telemetry.addData("blue", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
