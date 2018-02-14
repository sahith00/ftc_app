package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by sahith on 12/10/17.
 */
public class teleOp extends LinearOpMode{
    DcMotor fr, fl, br, bl;
    DcMotor lift;
    DcMotor rintake, lintake;

    Servo cat, knock;
    ColorSensor jewelSensor;

    Servo rflip, lflip, stopper;

    final static double CAT_STOW = 0.85944444444444444444;
    final static double KNOCK_STOW = .359444444444444444444445;

    final static double STOPPER_STOP = 0.959444444444444445;
    final static double STOPPER_DEPOSIT = 0.62000000000000001;
    final static double STOPPER_ZERO = 0.16944444444444452;
    final static double RFLIP_DEPOSIT = 0.789444444444444446;
    final static double RFLIP_ZERO = 0.18;
    final static double RFLIP_GRAB = 0.06;
    final static double LFLIP_DEPOSIT = 0.0294444444444444444;
    final static double LFLIP_ZERO = 0.619444444444444444445;
    final static double LFLIP_GRAB = 0.739444444444444444446;

    final static double LEVEL_ONE = 0;
    final static double LEVEL_TWO = -372;
    final static double LEVEL_THREE = -770;
    double intakep;
    boolean lift_zero;
    double multiplier = 1.0;
    boolean switch1, switch2, switch3;
    double zero_encoder, t;
    double desired_lift_val, desired_lift_power, currentpos;

    ElapsedTime ctime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        lift = hardwareMap.dcMotor.get("lift");

        rintake = hardwareMap.dcMotor.get("rintake");
        lintake = hardwareMap.dcMotor.get("lintake");

        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");
        stopper = hardwareMap.servo.get("stopper");

        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //so that lift can hold its position
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        grab(); //inits flipper to grab position to prepare for grabbing
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        lift.setPower(0);
        rintake.setPower(0);
        lintake.setPower(0);
        cat.setPosition(CAT_STOW);
        knock.setPosition(KNOCK_STOW);

        intakep = .0;
        lift_zero = true;
        zero_encoder = 0;
        desired_lift_val = 0;
        desired_lift_power = 0;
        currentpos = 0;
        switch1 = true;
        switch2 = false;
        switch3 = false;
        t = 0;

        waitForStart();
        while(opModeIsActive()) {
            cat.setPosition(CAT_STOW);
            knock.setPosition(KNOCK_STOW);
            //-----------------------------------------------------------------------------
            // DRIVE ROBOT
            mecanum(gamepad1.left_stick_y, -(gamepad1.left_stick_x), -(gamepad1.right_stick_x), multiplier);
            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT
            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            grabGlyph();
            moveLift();
            flip();
            //-----------------------------------------------------------------------------
            // TELEMETRY
            telemetry.addData("Lift Encoder Count: ", lift.getCurrentPosition());
            telemetry.addData("Lift Power", lift.getPower());
            telemetry.addData("Intake power", intakep);
            telemetry.update();
            //-----------------------------------------------------------------------------
        }
    }

    public void grabGlyph() {
        if (gamepad2.right_bumper) {
            intakep = 1.0;
        }
        if (gamepad2.left_bumper) {
            intakep = .0;
        }
        if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            runIntake(-0.65 * Math.signum(gamepad2.right_trigger), -0.65 * Math.signum(gamepad2.left_trigger));
        }
        else {
            runIntake(intakep, intakep);
        }
    }

    public void moveLift() {
        if (gamepad2.right_stick_y < 0) {
            desired_lift_power = .7;
            desired_lift_val = desired_lift_val - 1000; //Always keep running at power when joystick is moved
            switch1 = true;
            switch2 = false;
            switch3 = true;
        } else if (gamepad2.right_stick_y > 0) {
            desired_lift_power = .7;
            desired_lift_val = desired_lift_val + 1000;
            switch1 = true;
            switch2 = false;
            switch3 = true;
        } else {
            if (switch3) {
                currentpos = lift.getCurrentPosition();
                switch3 = false;
            }
            if (switch1) { //if we aren't moving our lift at all, have it stay at its last current position
                //   desired_lift_power = .0;
                desired_lift_val = currentpos;
            }
            if (gamepad2.a) {
                desired_lift_power = .7;
                desired_lift_val = zero_encoder + LEVEL_ONE;
                switch1 = false; //stop making lift stay at its current position
                switch2 = true;
                t = ctime.milliseconds(); //mark time
            }
            if (gamepad2.y) {
                zero();
                desired_lift_power = .7;
                desired_lift_val = zero_encoder + LEVEL_THREE;;
                switch1 = false;
                switch2 = true;
                t = ctime.milliseconds();
            }
            if (gamepad2.x) {
                zero();
                desired_lift_power = .7;
                desired_lift_val = zero_encoder + LEVEL_TWO;
                switch1 = false;
                switch2 = true;
                t = ctime.milliseconds();
            }
            if (switch2) {
                if (ctime.milliseconds() < t + 1100) {//account for time needed for lift to get to preset value
                    switch1 = false;
                } else {
                    switch1 = true;
                    switch2 = false;
                    switch3 = true;
                }
            }
            if (gamepad2.b) {
                zero_encoder = lift.getCurrentPosition();
            }
        }
        lift.setPower(desired_lift_power);
        lift.setTargetPosition((int) desired_lift_val);
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
    }
    public void zero() {
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
        stopper.setPosition(STOPPER_ZERO);
    }
    public void deposit() {
        rflip.setPosition(RFLIP_DEPOSIT);
        lflip.setPosition(LFLIP_DEPOSIT);
        stopper.setPosition(STOPPER_DEPOSIT);
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
            fl.setPower(multiplier * (v1/max));
            fr.setPower(multiplier * (v2/max));
            bl.setPower(multiplier * (v3/max));
            br.setPower(multiplier * (v4/max));
        } else {
            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);
        }
    }
}