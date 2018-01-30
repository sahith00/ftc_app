package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by sahith on 12/24/17.
 */

public class colorSensorTest extends LinearOpMode {
    ModernRoboticsI2cColorSensor colorSensor1, colorSensor2, colorSensor3, colorSensor4;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor1 = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "rightLineSensor");
        colorSensor2 = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"frontMiddleLineSensor");
        colorSensor3 = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"centerLineSensor");
        colorSensor4 = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"leftLineSensor");

        waitForStart();
        while(opModeIsActive()) {
            //colorSensor.enableLed(false);
            telemetry.addData("rightLineSensor Red Value", colorSensor1.red());
            telemetry.addData("rightLineSensor Blue Value", colorSensor1.blue());
            telemetry.addData("frontMiddleLineSensor Red Value", colorSensor2.red());
            telemetry.addData("frontMiddleLineSensor Blue Value", colorSensor2.blue());
            telemetry.addData("centerLineSensor Red Value", colorSensor3.red());
            telemetry.addData("centerLineSensor Blue Value", colorSensor3.blue());
            telemetry.addData("leftLineSensor Red Value", colorSensor4.red());
            telemetry.addData("leftLineSensor Blue Value", colorSensor4.blue());
            telemetry.update();
        }
    }
}
