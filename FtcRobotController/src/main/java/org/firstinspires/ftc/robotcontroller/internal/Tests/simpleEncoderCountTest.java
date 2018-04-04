package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by sahith on 1/26/18.
 */

public class simpleEncoderCountTest extends LinearOpMode {
    DcMotor frdrive, fldrive, brdrive, bldrive;

    final static double FORWARD_POWER = 0.3;
    int ENCODER_TICKS = 2000;
    double desired_pos = ENCODER_TICKS;

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bldrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        brdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        fldrive.setPower(0.0);
        frdrive.setPower(0.0);
        bldrive.setPower(0.0);
        brdrive.setPower(0.0);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_down) {
                sleep(250);
                ENCODER_TICKS -= 100;
            }
            if (gamepad1.dpad_up) {
                sleep(250);
                ENCODER_TICKS += 100;
            }
            if (gamepad1.b) {
                sleep(250);
                ENCODER_TICKS = -ENCODER_TICKS;
            }
            if (gamepad1.a) {
                desired_pos += fldrive.getCurrentPosition();
                sleep(1000);
                testSideTicks(desired_pos);
            }
            telemetry.addData("Encoder count:", fldrive.getCurrentPosition());
            telemetry.update();
        }
    }

    public void testSideTicks(double position) {
        while(fldrive.getCurrentPosition() < ENCODER_TICKS) {
            fldrive.setPower(-FORWARD_POWER);
            frdrive.setPower(FORWARD_POWER);
            bldrive.setPower(FORWARD_POWER);
            brdrive.setPower(-FORWARD_POWER);
            telemetry.addData("Current position", fldrive.getCurrentPosition());
            telemetry.update();
        }
        fldrive.setPower(0.0);
        frdrive.setPower(0.0);
        bldrive.setPower(0.0);
        brdrive.setPower(0.0);
    }
}