package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by sahith on 11/2/17.
 */
public class encoderCountTest extends LinearOpMode{
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
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        testMotorTicks(fldrive, frdrive, bldrive, brdrive);

        testSideTicks(fldrive, frdrive, bldrive, brdrive);

        telemetry.addData("flmotor", fldrive.getCurrentPosition());
        telemetry.addData("frmotor", frdrive.getCurrentPosition());
        telemetry.addData("blmotor", bldrive.getCurrentPosition());
        telemetry.addData("brmotor", brdrive.getCurrentPosition());

        telemetry.update();
    }

    public void mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, double multiplier) {
        double joyly = gamepad1.left_stick_y;
        double joylx = -(gamepad1.left_stick_x);
        double joyrx = -(gamepad1.right_stick_x);
//        double bumpers = gamepad1.right_trigger-gamepad1.left_trigger;
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joyly, joylx);
        double v0 = joyrx;
        //v0 = bumpers;
        double v1 = vd*Math.sin(theta + (Math.PI / 4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))-v0; //fr
        double v3 = vd*Math.cos(theta + (Math.PI / 4))+v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));

        if (max != 0) {
            fl.setPower(multiplier * (v1/max));
            fr.setPower(multiplier * (v2/max));
            bl.setPower(multiplier * (v3/max));
            br.setPower(multiplier * (v4/max));
        } else {
            fl.setPower(0.0);
            fr.setPower(0.0);
            bl.setPower(0.0);
            br.setPower(0.0);
        }
    }

    public void testMotorTicks(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        while(motor1.getCurrentPosition() < 200) {
            motor1.setPower(0.8);
            motor2.setPower(0.8);
            motor3.setPower(0.8);
            motor4.setPower(0.8);
        }
        while(motor1.getCurrentPosition() < 300) {
            motor1.setPower(0.6);
            motor2.setPower(0.6);
            motor3.setPower(0.6);
            motor4.setPower(0.6);
        }
        while(motor1.getCurrentPosition() < 350) {
            motor1.setPower(0.5);
            motor2.setPower(0.5);
            motor3.setPower(0.5);
            motor4.setPower(0.5);
        }
        while(motor1.getCurrentPosition() < 375) {
            motor1.setPower(0.3);
            motor2.setPower(0.3);
            motor3.setPower(0.3);
            motor4.setPower(0.3);
        }
        while(motor1.getCurrentPosition() < 390) {
            motor1.setPower(0.2);
            motor2.setPower(0.2);
            motor3.setPower(0.2);
            motor4.setPower(0.2);
        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    public void testSideTicks(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        double joyly = 0;
        double joylx = -1;
        double joyrx = 0;
//        double bumpers = gamepad1.right_trigger-gamepad1.left_trigger;
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joyly, joylx);
        double v0 = joyrx;
        //v0 = bumpers;
        double v1 = vd*Math.sin(theta + (Math.PI / 4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))-v0; //fr
        double v3 = vd*Math.cos(theta + (Math.PI / 4))+v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));

        if (max != 0) {
            while(motor1.getCurrentPosition() < 200) {
                motor1.setPower(0.8 * (v1 / max));
                motor2.setPower(0.8 * (v2 / max));
                motor3.setPower(0.8 * (v3 / max));
                motor4.setPower(0.8 * (v4 / max));
            }
            while(motor1.getCurrentPosition() < 300) {
                motor1.setPower(0.6 * (v1 / max));
                motor2.setPower(0.6 * (v2 / max));
                motor3.setPower(0.6 * (v3 / max));
                motor4.setPower(0.6 * (v4 / max));
            }
            while(motor1.getCurrentPosition() < 350) {
                motor1.setPower(0.5 * (v1 / max));
                motor2.setPower(0.5 * (v2 / max));
                motor3.setPower(0.5 * (v3 / max));
                motor4.setPower(0.5 * (v4 / max));
            }
            while(motor1.getCurrentPosition() < 375) {
                motor1.setPower(0.3 * (v1 / max));
                motor2.setPower(0.3 * (v2 / max));
                motor3.setPower(0.3 * (v3 / max));
                motor4.setPower(0.3 * (v4 / max));
            }
            while(motor1.getCurrentPosition() < 390) {
                motor1.setPower(0.2 * (v1 / max));
                motor2.setPower(0.2 * (v2 / max));
                motor3.setPower(0.2 * (v3 / max));
                motor4.setPower(0.2 * (v4 / max));
            }
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }
    }

}
