package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by vulcanrobotics8375 on 4/18/18.
 */

public class autoRed285 extends autoMethods {

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

        stow();
        driveDist(15.5, 0.4, false);
        sleep(500);
        /*turn(90, 5);
        driveDist(-4, -0.3, false);*/
        strafeDist(-3.5, -0.5);
        turn(90, .5);
        colorDistExtendFar();
        doImage("RED", imageDetected, 8.5+2, 46.5+2, 31+2, false);
    }
}
