package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 2/12/18.
 */

public class autoBlue2 extends autoMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        autoT = runtime.milliseconds();

        knock.setPosition(KNOCK_CENTER);
        extendstopper.setPosition(EXTENDSTOPPER_STOP);
        stopper.setPosition(STOPPER_STOW);

        jewelAuto();
        imageDetected = doVuforia();

        stow();
        driveDistance(-12, -0.5, false);
       /* turn(-90, 2.5);
        driveDist(4, 0.3, false);*/
        // sleep(1000);
        sleep(500);
        strafeDist(-5.5, -0.5);
        turn(-90, 1);
        colorDistExtendFar();
        doImage("BLUE", imageDetected, 133.5 - 0.5 - 1.5, 168 - 0.5 - 1.5, 149 - 0.5 - 1.5, false);

        glyphAutoFarBlue(imageDetected,-168);
        while (opModeIsActive()) {}
    }
}