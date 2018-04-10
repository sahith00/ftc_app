package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 2/12/18.
 */

public class autoRed2 extends autoMethods {

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

        jewelAuto("RED");
        imageDetected = doVuforia();

        driveDistance(10, 0.5);
        turn(90, 2.5);
        sleep(500);
        doImage("RED", imageDetected, 10, 40, 25);
    }
}