package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 2/12/18.
 */

public class autoBlue2 extends autoMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        resetAngles();

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        rintake.setPower(0);
        lintake.setPower(0);

        knock.setPosition(KNOCK_STOW);
        sleep(250);
        cat.setPosition(CAT_STOW);
        sleep(250);
        lig.setPosition(LIG_STOW);
        sleep(250);

        zero();

        setUpVuforia();

        waitForStart();

        autoT = runtime.milliseconds();

        lig.setPosition(LIG_HALF_STOW);
        sleep(250);

        jewelAuto("BLUE");
        imageDetected = doVuforia();



//        drive(-7.5, -0.4);
//        sleep(500);
//        turn(90, 3.5);
//        sleep(500);
//        drive(7, 0.4);
//        sleep(500);
//        doImage(imageDetected, 153, 203, 178);
//        glyphAutoFar(203, 213);
    }
}