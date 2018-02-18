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

        grab();

        waitForStart();

        grabGlyph(1.0);
        driveBackward(-24, -0.3);
        sleep(4000);
        driveBackward(-7, -0.3);
        sleep(1000);
        zero();
        sleep(500);
        turn(0, 3.5);
        driveForward(29, 0.5);
        sleep(500);
        outtake();
        sleep(500);
        driveBackward(-3, -0.3);
        driveForward(3, 0.3);
        driveBackward(-4, -0.3);
    }
}