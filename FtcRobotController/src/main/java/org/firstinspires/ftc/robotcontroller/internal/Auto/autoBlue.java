package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 12/10/17.
 */

public class autoBlue extends autoMethods {

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

        closeAutoBlue85();
        glyphAutoClose("L", "BLUE", -124, -96, -112, -90);
        while(opModeIsActive()){}
    }
}