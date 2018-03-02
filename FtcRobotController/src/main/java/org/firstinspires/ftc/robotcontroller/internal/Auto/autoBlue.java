package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 12/10/17.
 */

public class autoBlue extends autoMethods {

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

        jewelAuto("BLUE");
        imageDetected = doVuforia();
        driveBackward(-25, -0.4);
        sleep(1000);
        doImage(imageDetected, -116, -65, -90);
        //glyphAuto(-90, -65);
    }
}