package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by sahith on 3/24/18.
 */

public class recordAndPlayTeleTest extends LinearOpMode {
    DcMotor fr, fl, br, bl;
    DcMotor rintake, lintake;
    DcMotor lift;

    Servo rflip, lflip, stopper;

    private ElapsedTime timer;
    private boolean recording;
    private String state;
    public BufferedWriter bw;

    double intakep;
    boolean grabFlip;

    final static double STOPPER_STOP = 0.0;
    final static double STOPPER_STOW = 1.0;
    final static double RFLIP_DEPOSIT = 0.07;
    final static double RFLIP_ZERO = 0.640000000000000000001;
    final static double RFLIP_GRAB = 0.679444444444444444445;
    final static double LFLIP_DEPOSIT = 0.91944444444444444445;
    final static double LFLIP_ZERO = 0.319444444444444444445;
    final static double LFLIP_GRAB = 0.269444444444444444443;

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        rintake = hardwareMap.dcMotor.get("rintake");
        lintake = hardwareMap.dcMotor.get("lintake");
        lift = hardwareMap.dcMotor.get("lift");

        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");
        stopper = hardwareMap.servo.get("stopper");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rintake.setDirection(DcMotorSimple.Direction.REVERSE);
        lintake.setDirection(DcMotorSimple.Direction.FORWARD);

        grab(); //inits flipper to grab position to prepare for grabbing
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        rintake.setPower(0);
        lintake.setPower(0);
        lift.setPower(0);

        intakep = .0;
        grabFlip = false;

        timer = new ElapsedTime();
        recording = false;

        waitForStart();

        while(opModeIsActive()) {
            if (!recording && gamepad1.start) {
                while (gamepad1.start) ;

                recording = true;
                timer.reset();
                state = toState(gamepad1);

                try {
                    bw = new BufferedWriter(new FileWriter(
                            new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS),
                                    "blue_left.txt")));
                } catch (IOException ioe) {
                    ioe.printStackTrace();
                } catch (NullPointerException npe) {
                    npe.printStackTrace();
                }
            }

            if (recording) {
                if (timer.milliseconds() >= 30000) {
                    recording = false;

                    try {
                        if (bw != null) bw.close();
                    } catch (IOException ioe) {
                    }
                } else if (!toState(gamepad1).equals(state)) {
                    try {
                        bw.write(state + timeToString(timer.seconds()) + "\n");
                    } catch (IOException ioe) {
                    }
                    state = toState(gamepad1);
                }

                doTele();

                //wait 0.05 second
                sleep(50);
            } else {
                doTele();
            }
        }
    }

    public char doubleToChar(double d) {
        for (int i = 1; i <= 26; i++) {
            if (d <= i / 13. - 1) {
                return (char) ('@' + i);
            }
        }
        return 'A';
    }

    public String timeToString(double d) {
        int c = 58; //A-z; use 26 for A-Z
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < c; j++) {
                if (d <= 30. / (c * c) * (i * c + j)) {
                    return "" + (char) ('A' + i) + (char) ('A' + j);
                }
            }
        }
        return "" + (char) ('@' + c) + (char) ('@' + c); //should be "zz"
    }

    public String toState(Gamepad gp) {
        String output = "";
        int temp = 0;
        if (gp.a) temp += (1 << 0);
        if (gp.b) temp += (1 << 1);
        if (gp.x) temp += (1 << 2);
        if (gp.y) temp += (1 << 3);
        if (gp.dpad_down) temp += (1 << 4);
        if (gp.dpad_left) temp += (1 << 5);
        if (gp.dpad_right) temp += (1 << 6);
        if (gp.dpad_up) temp += (1 << 7);
        if (gp.left_bumper) temp += (1 << 8);
        if (gp.left_stick_button) temp += (1 << 9);
        if (gp.left_trigger > 0.1) temp += (1 << 10);
        if (gp.right_bumper) temp += (1 << 11);
        if (gp.right_stick_button) temp += (1 << 12);
        if (gp.right_trigger > 0.1) temp += (1 << 13);
        output += temp;

        while (output.length() < 5) output = "0" + output;

        output += doubleToChar(gp.left_stick_x);
        output += doubleToChar(gp.left_stick_y);
        output += doubleToChar(gp.right_stick_x);
        output += doubleToChar(gp.right_stick_y);

        return output;
    }

    public void mecanum(double joyly, double joylx, double joyrx, double multiplier) {
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joyly, joylx);
        double v0 = joyrx;
        //v0 = bumpers;
        double v1 = vd*Math.sin(theta+(Math.PI/4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))+v0; //fr
        double v3 = vd*Math.cos(theta+(Math.PI/4))-v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));
        if (max > 1) {
            fl.setPower(multiplier * (Math.pow(v1/max, 3.0)));
            fr.setPower(multiplier * (Math.pow(v2/max, 3.0)));
            bl.setPower(multiplier * (Math.pow(v3/max, 3.0)));
            br.setPower(multiplier * (Math.pow(v4/max, 3.0)));
        } else {
            fl.setPower(multiplier*(Math.pow(v1, 3.0)));
            fr.setPower(multiplier*(Math.pow(v2, 3.0)));
            bl.setPower(multiplier*(Math.pow(v3, 3.0)));
            br.setPower(multiplier*(Math.pow(v4, 3.0)));
        }
    }

    public void flip() {
        if (gamepad1.x || gamepad2.dpad_left) { //zero position, when flipper is parallel to ground
            zero();
        }
        if (gamepad1.a || gamepad2.dpad_down) { //grab position, when we're picking up cubes
            grab();
        }
        if (gamepad1.y || gamepad2.dpad_up) { //deposit position, when we're depositing cubes
            deposit();
        }
    }

    public void runIntake(double rpower, double lpower) {
        rintake.setPower(rpower);
        lintake.setPower(lpower);
    }

    public void grab() {
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
        stopper.setPosition(STOPPER_STOP);
        grabFlip = false;
    }
    public void zero() {
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
        stopper.setPosition(STOPPER_STOW);
        grabFlip = true;
    }
    public void deposit() {
        rflip.setPosition(RFLIP_DEPOSIT);
        lflip.setPosition(LFLIP_DEPOSIT);
        stopper.setPosition(STOPPER_STOW);
        grabFlip = true;
    }

    public void grabGlyph() {
        if (gamepad2.right_bumper) {
            intakep = 1.0;
        }
        if (gamepad2.left_bumper) {
            intakep = .0;
        }
        if (gamepad2.right_trigger > 0.85 || gamepad2.left_trigger > 0.85 || gamepad1.right_trigger > 0.1) {
            runIntake(-1.0 * Math.signum(gamepad2.right_trigger)
                    , -1.0 * Math.signum(gamepad2.left_trigger));
        }
        else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            runIntake(-0.5 * Math.signum(gamepad2.right_trigger)
                    , -0.5 * Math.signum(gamepad2.left_trigger));
        }
        else {
            runIntake(intakep, intakep);
        }
    }

    public void doTele() {
        mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, -(gamepad1.right_stick_x), 0.7);
        grabGlyph();
        flip();
        lift.setPower(gamepad2.right_stick_y);
    }
}