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

public class autoBlue extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        flipper = new Flipper(hardwareMap);
        vuforia = new Vuforia(0, V_KEY);
        intake = new Intake(hardwareMap);
        jewel = new Jewel(hardwareMap);

        drivetrain.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        drivetrain.showPID();

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        jewelAuto();
        imageDetected = vuforia.doVuforia();
        drivetrain.driveBackward(-24, -0.4);             //-22 = directly to center
        sleep(2000);
        doImage(imageDetected);
    }

    //IMAGE FUNCTIONS-----------------------------------------------------------------------------
    public void doImage(String image) {
        // check for robot at the top of the triangle
        if (image.equals("R")) {
            drivetrain.turn(-120, 3);
            drivetrain.driveForward(6, 0.3);
            outtake();
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
            sleep(1000);
            drivetrain.driveForward(7, 0.3);
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
        } else if (image.equals("L")) {
            drivetrain.turn(-74, 3);
            drivetrain.driveForward(6, 0.3);
            outtake();
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
            sleep(1000);
            drivetrain.driveForward(7, 0.3);
            sleep(500);
            drivetrain.driveBackward(-5, -0.3);
        } else {
            drivetrain.turn(-98, 3);
            drivetrain.driveForward(4, 0.3);
            outtake();
            sleep(500);
            drivetrain.driveBackward(-4, -0.3);
            sleep(1000);
            drivetrain.driveForward(5, 0.3);
            sleep(500);
            drivetrain.driveBackward(-4, -0.3);
        }
    }
    //----------------------------------------------------------------------------------------------

    //JEWEL FUNCTION--------------------------------------------------------------------------------
    public void jewelAuto() {
        extend();
        sleep(1000);
        if (jewel.isBlue()) {
            right();
            jewel.knockCenter();
            sleep(250);
            stow();
        }
        else {
            left();
            jewel.knockCenter();
            sleep(250);
            stow();
        }
    }


    public void stow() {
        jewel.halfCat();
        sleep(250);
        jewel.stowKnock();
        sleep(250);
        jewel.stowCat();
        sleep(250);
    }

    public void extend() {
        jewel.halfCat();
        sleep(250);
        jewel.knockCenter();
        sleep(250);
        jewel.extendCat();
        sleep(250);
    }

    public void right() {
        jewel.knockRight();
        sleep(250);
    }

    public void left() {
        jewel.knockLeft();
        sleep(250);
    }
    //----------------------------------------------------------------------------------------------

    //GLYPH FUNCTIONS-------------------------------------------------------------------------------
    public void outtake() {
        flipper.deposit();
        sleep(500);
    }
    //----------------------------------------------------------------------------------------------
}