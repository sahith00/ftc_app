package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Drivetrain;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Flipper;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Intake;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Jewel;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Vuforia;

/**
 * Created by sahith on 12/10/17.
 */

public class autoRed extends LinearOpMode {
    Drivetrain drivetrain;
    Flipper flipper;
    Intake intake;
    Vuforia vuforia;
    Jewel jewel;

    public static final String V_KEY = "Aelee1z/////AAAAGSOzCHm55k0ymtB98J5uxqsJ3VZ0L7SyL2P9mk" +
            "wAcASPo5jphdOuL+kEucaxbqfA3GfMKZbO9/7zChpv5oYcZ+yc/T6cl7LjsJVETcd2kf+6W1cby1xBzFB" +
            "wuHi0wHKvxx+PmsKhwA0dLReiaVIkev7aJ8CrAhDfPfkeT8HMXNRvoJ3tkiAkfc9ONcNxNt4XzbXTcslg" +
            "8/+xgHw/q7yhnuWy1hgPizTCcIQr+oWRuukFDC228GuWhTiHDGFqHWMd0dublyymuEgSxpSByagvaHJ73" +
            "/mJbfRCTcBJgmvLmAzv86N6vyycC4pReQxHn8djz8QCicZnEbz9CM10HcaRxqftMFNv0iKnlDH85NJrnyMw";

    String imageDetected;

    double dist = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        flipper = new Flipper(hardwareMap);
        intake = new Intake(hardwareMap);
        vuforia = new Vuforia(0, V_KEY);
        jewel = new Jewel(hardwareMap);

        drivetrain.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drivetrain.fr.setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrain.br.setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrain.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.bl.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain.fr.setPower(0);
        drivetrain.fl.setPower(0);
        drivetrain.br.setPower(0);
        drivetrain.bl.setPower(0);
        intake.rintake.setPower(0);
        intake.lintake.setPower(0);

        jewel.stowCat();

        flipper.zero();

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        imageDetected = vuforia.doVuforia();
        jewelAuto();
        drivetrain.driveForward(15 - dist, 0.4);
        sleep(2000);
        doImage(imageDetected);
    }

    //IMAGE FUNCTIONS-----------------------------------------------------------------------------
    public void doImage(String image) {
        // check for robot at the top of the triangle
        if (image.equals("R")) {
            drivetrain.turn(-124, 3.5);
            drivetrain.driveForward(6, 0.3);
            outtake();
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
            sleep(500);
            drivetrain.driveForward(7, 0.3);
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
        } else if (image.equals("L")) {
            drivetrain.turn(-70, 3.5);
            drivetrain.driveForward(6, 0.3);
            outtake();
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
            sleep(500);
            drivetrain.driveForward(7, 0.3);
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
        } else {
            drivetrain.turn(-97, 3.5);
            drivetrain.driveForward(4, 0.3);
            outtake();
            sleep(500);
            drivetrain.driveBackward(-4, -0.3);
            sleep(500);
            drivetrain.driveForward(5, 0.3);
            sleep(500);
            drivetrain.driveBackward(-4, -0.3);
        }
    }
    //----------------------------------------------------------------------------------------------

    //JEWEL FUNCTION--------------------------------------------------------------------------------
    public void jewelAuto() {
        jewel.extendCat();
        sleep(1000);
        if (jewel.isRed()) {
            dist = 2;
            drivetrain.driveForward(2, 0.35);
            sleep(2000);
            jewel.stowCat();
            sleep(2000);
        }
        else {
            dist = -5;
            drivetrain.driveBackward(-3, -0.35);
            sleep(500);
            jewel.stowCat();
            sleep(500);
            drivetrain.driveForward(10, 1);
            sleep(500);
        }
    }
    //----------------------------------------------------------------------------------------------

    //GLYPH FUNCTIONS-------------------------------------------------------------------------------
    public void outtake() {
        flipper.deposit();
        sleep(500);
    }
    //----------------------------------------------------------------------------------------------
}