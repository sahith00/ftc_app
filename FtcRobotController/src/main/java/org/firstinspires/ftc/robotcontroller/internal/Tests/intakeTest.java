package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by sahith on 1/3/18.
 */

public class intakeTest extends LinearOpMode {
    DcMotor rgrab, lgrab;
    boolean forward = true;

    @Override
    public void runOpMode() throws InterruptedException {
        rgrab = hardwareMap.dcMotor.get("rgrab");
        lgrab = hardwareMap.dcMotor.get("lgrab");
        rgrab.setPower(0.0);
        lgrab.setPower(0.0);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                sleep(250);
                forward = true;
            }
            if (gamepad1.b) {
                sleep(250);
                forward = false;
            }
            if (forward) {
                rgrab.setPower(gamepad1.right_trigger);
                lgrab.setPower(gamepad1.left_trigger);
            }
            if (!forward) {
                rgrab.setPower(-gamepad1.right_trigger);
                lgrab.setPower(-gamepad1.left_trigger);
            }

            telemetry.addData("Right power", rgrab.getPower());
            telemetry.addData("Left power", lgrab.getPower());
            telemetry.update();
        }
    }
}
