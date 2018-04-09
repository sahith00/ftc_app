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

        knock.setPosition(KNOCK_RIGHT);
        sleep(250);
        cat.setPosition(CAT_STOW);
        sleep(250);
        lig.setPosition(LIG_STOW);
        sleep(250);
        stopper.setPosition(STOPPER_STOW);
        sleep(250);
        extendstopper.setPosition(EXTENDSTOPPER_STOW);
        sleep(250);

        zero();

        setUpVuforia();

        waitForStart();

        autoT = runtime.milliseconds();

        knock.setPosition(KNOCK_CENTER);
        sleep(250);
        lig.setPosition(LIG_HALF_STOW);
        sleep(250);
        extendstopper.setPosition(EXTENDSTOPPER_STOP);
        sleep(250);
        stopper.setPosition(STOPPER_STOP);
        sleep(250);

        jewelAuto("BLUE");
        //imageDetected = doVuforia();
        //drive up to cryptobox with ir sensor
        doImage("BLUE", "C", -80, -50, -65);


    }
}