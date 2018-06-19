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
        sleep(200);
        extendstopper.setPosition(EXTENDSTOPPER_STOP);
        sleep(250);


        jewelAuto();
        imageDetected = doVuforia();

        farAutoBlue85();
        double glyphangle = 0;
        if (imageDetected.equals("C")) {
            glyphangle = -165; //deposit in right column
        } else {
            glyphangle = -155; //deposit in center column
        }
        glyphAutoFarBlue(imageDetected, glyphangle);

    }
}