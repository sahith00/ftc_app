package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by sahith on 12/1/17.
 */

public class NavX_Zero extends LinearOpMode {
    NavX navX;

    @Override
    public void runOpMode() throws InterruptedException {
        navX = new NavX(hardwareMap);

        waitForStart();
        navX.start();
        while (opModeIsActive()) {
            telemetry.addData("yaw", navX.yaw);
        }

    }
}