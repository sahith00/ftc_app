package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by vulcanrobotics8375 on 4/18/18.
 */

public class autoRed85 extends autoMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        autoT = runtime.milliseconds();

        knock.setPosition(KNOCK_CENTER);
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
        doImage("RED", imageDetected, -78, -47.5, -64, true);
        sleep(500);
        driveDistance(6.5, 0.4, true);
        sleep(500);
        driveDistance(-5, -0.4, true);
    }
}
