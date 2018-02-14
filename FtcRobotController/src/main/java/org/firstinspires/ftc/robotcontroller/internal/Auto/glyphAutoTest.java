package org.firstinspires.ftc.robotcontroller.internal.Auto;

/**
 * Created by sahith on 1/19/18.
 */

public class glyphAutoTest extends autoMethods {

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

        zero();

        waitForStart();

        grab();
        sleep(500);
        grabGlyph(-0.7);
        driveBackward(-18, -0.3);
        sleep(3000);
        turn(15, 3.5);
        sleep(500);
        turn(-15, 3.5);
        sleep(1000);
        zero();
        sleep(500);
        turn(0, 3.5);
        driveForward(20, 0.5);
        sleep(500);
        outtake();
    }
}