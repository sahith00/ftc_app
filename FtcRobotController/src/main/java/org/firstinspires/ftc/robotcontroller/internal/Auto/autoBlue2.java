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
        lig.setPosition(.12944444444444444447 + 0.3);
        jewelAuto("BLUE");
        imageDetected = doVuforia();
        driveBackward(-7.5, -0.4);
        sleep(1000);
        turn(90, 3.5);
        sleep(1000);
        driveForward(9, 0.4);
        sleep(1000);
        doImage(imageDetected, 153, 203, 178);
    }
}