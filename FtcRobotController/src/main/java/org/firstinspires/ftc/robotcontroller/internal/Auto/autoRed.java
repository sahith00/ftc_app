package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 12/10/17.
 */

public class autoRed extends autoMethods {

        @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        autoT = runtime.milliseconds();

        knock.setPosition(KNOCK_CENTER);
        sleep(200);
        lig.setPosition(LIG_HALF_STOW);
        sleep(200);
        extendstopper.setPosition(EXTENDSTOPPER_STOP);
        sleep(250);
        stopper.setPosition(STOPPER_STOW);
        sleep(250);

        jewelAuto();
        imageDetected = doVuforia();
        colorExtendClose();
        driveDist(2.5, .4, false);
        //drive up to cryptobox with ir sensor
        doImage("RED", imageDetected, -78, -50.5, -64, true);
        while (opModeIsActive()) {}
    }
}