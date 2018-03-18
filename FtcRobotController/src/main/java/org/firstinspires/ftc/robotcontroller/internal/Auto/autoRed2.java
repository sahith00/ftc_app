package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 2/12/18.
 */

public class autoRed2 extends autoMethods {

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

        jewelAuto("RED");
        imageDetected = doVuforia();





//        drive(11, 0.4);
//        sleep(500);
//        turn(-90, 3.5);
//        sleep(500);
//        double tempt = runtime.milliseconds();
//        fr.setPower(0.4);
//        fl.setPower(0.4);
//        br.setPower(0.4);
//        bl.setPower(0.4);
//        while(tempt + 5000 > runtime.milliseconds()) {
//        }
//        fr.setPower(0.0);
//        fl.setPower(0.0);
//        br.setPower(0.0);
//        bl.setPower(0.0);
//        sleep(500);
//        drive(-18, -0.4);
//        sleep(500);
//        doImage(imageDetected, -25, 25, 0);
//        glyphAutoFar(25, -35);
    }
}