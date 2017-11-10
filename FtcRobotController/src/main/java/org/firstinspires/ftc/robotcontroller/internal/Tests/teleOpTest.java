package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by sahith on 11/10/17.
 */
public class teleOpTest extends LinearOpMode {
    DcMotor frdrive, fldrive, brdrive, bldrive;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");

        fldrive.setPower(0.0);
        frdrive.setPower(0.0);
        bldrive.setPower(0.0);
        brdrive.setPower(0.0);

        waitForStart();
        while(opModeIsActive()) {

            fldrive.setPower(gamepad1.left_stick_y);
            frdrive.setPower(gamepad1.right_stick_y);
            bldrive.setPower(gamepad1.left_stick_y);
            brdrive.setPower(gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}
