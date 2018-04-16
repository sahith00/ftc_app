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

        jewelAuto();
        imageDetected = doVuforia();

        stow();
        driveDist(14, 0.4, false);
        sleep(500);
        /*turn(90, 5);
        driveDist(-4, -0.3, false);*/
        strafeDist(-3.5, -0.5);
        turn(90, .5);
        colorDistExtendFar();
        doImage("RED", imageDetected, 8.5,46.5, 31, false);
        glyphAutoFar(imageDetected, "RED", -25);
        while (opModeIsActive()) {}
    }
}