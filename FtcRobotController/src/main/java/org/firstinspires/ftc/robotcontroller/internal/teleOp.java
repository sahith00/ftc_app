package org.firstinspires.ftc.robotcontroller.internal;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sahith on 12/10/17.
 */
public class teleOp extends LinearOpMode{
    DcMotor fr, fl, br, bl;
    DcMotor lift, relicLift;
    DcMotor rintake, lintake;

    Servo cat, knock;
    Servo rflip, lflip, stopper;
    Servo lig, claw;

    BNO055IMU imu;
    Orientation lastAngles;

    final static double CAT_STOW = 0.85944444444444444444; // during teleop after intialization subtract 0.01
    final static double KNOCK_STOW = .42;

    final static double STOPPER_STOP = 0.0;
    final static double STOPPER_STOW = 0.5;
    final static double RFLIP_DEPOSIT = 0.069444444444444448;
    final static double RFLIP_ZERO = 0.629444444444444444445;
    final static double RFLIP_GRAB = 0.679444444444444444445;
    final static double LFLIP_DEPOSIT = 0.93;
    final static double LFLIP_ZERO = 0.359444444444444444445;
    final static double LFLIP_GRAB = 0.299444444444444444446;

    final static double LIG_STOW = .01999999999999994;
    final static double LIG_GRAB = .8094444444444444444 + 0.04;//.899444444444444444445;
    final static double LIG_HALF_STOW = 0.35944444444444445;
    final static double CLAW_STOW = 0.279999999999999999997;
    final static double CLAW_OPEN = .799444444444444444444;
    final static double CLAW_GRAB = .249444444444444444;

    final static double FAR_ANGLE = 90;
    final static double CLOSE_ANGLE = 0;

    final static double LEVEL_ONE = 0;
    final static double LEVEL_TWO = -565;
    final static double LEVEL_THREE = -930;

    double t0, t1, t2, t3, t4, t5;
    double intakep;
    boolean lift_zero;
    double multiplier;
    boolean switch1, switch2, switch3;
    double zero_encoder;
    double desired_lift_val, currentpos, desired_lift_p;
    double stowPos;
    double clawPos;
    boolean grabFlip;
    boolean glyphMode;
    boolean farpid;
    boolean closepid;
    boolean liftpid;
    double desired_lift_pos;
    int position;

    double p_turn = .045;//0.008;
    double i_turn = .002; //.0045; //.003;
    double d_turn = .002; //.04 //.0045;
    double pT = 0;
    double pE = 0;
    double tE = 0;
    double pYaw = 0;

    double oldT = 0;
    double oldE = 0;
    double oldtE = 0;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime ctime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        lift = hardwareMap.dcMotor.get("lift");
        relicLift = hardwareMap.dcMotor.get("relicLift");

        rintake = hardwareMap.dcMotor.get("rintake");
        lintake = hardwareMap.dcMotor.get("lintake");

        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");
        stopper = hardwareMap.servo.get("stopper");

        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");

        lig = hardwareMap.servo.get("lig");
        claw = hardwareMap.servo.get("relicGrab");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //so that lift can hold its position
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        rintake.setDirection(DcMotorSimple.Direction.REVERSE);
        lintake.setDirection(DcMotorSimple.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        grab(); //inits flipper to grab position to prepare for grabbing
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        lift.setPower(0);
        relicLift.setPower(0);
        rintake.setPower(0);
        lintake.setPower(0);
        cat.setPosition(CAT_STOW);
        knock.setPosition(KNOCK_STOW);
        lig.setPosition(LIG_HALF_STOW);
        claw.setPosition(CLAW_STOW);

        clawPos = CLAW_STOW;
        multiplier = 1.0;
        intakep = .0;
        lift_zero = true;
        zero_encoder = 0;
        desired_lift_val = 0;
        currentpos = 0;
        stowPos = LIG_STOW;
        switch1 = false;
        switch2 = false;
        switch3 = false;
        t0 = 0;
        t1 = 0;
        t2 = 0;
        t3 = 0;
        t4 = 0;
        t5 = 0;
        grabFlip = false;
        glyphMode = true;
        farpid = false;
        closepid = false;
        liftpid = false;
        position = 0;
        desired_lift_pos = LEVEL_ONE;

        waitForStart();
        while(opModeIsActive()) {
            cat.setPosition(CAT_STOW-.01);
            knock.setPosition(KNOCK_STOW);

            if (gamepad1.b && (ctime.milliseconds() > (t1+250))) {
                t1 = ctime.milliseconds();
                glyphMode = !glyphMode;
            }
            //-----------------------------------------------------------------------------
            // DRIVE ROBOT
            if(glyphMode) {
                multiplier = Range.clip(Math.pow(1 - gamepad1.left_trigger, 2), 0.3, 1);
            }
            else {
                multiplier = 1.0;
                if (gamepad1.left_bumper && (ctime.milliseconds() > (t2+250))) {
                    t2 = ctime.milliseconds();
                    if (multiplier == 1.0) {
                        multiplier = 0.3;
                    }
                    else {
                        multiplier = 1.0;
                    }
                }
            }
            mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, -(gamepad1.right_stick_x), multiplier);
            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT
            if(!glyphMode) {
                relicLift.setPower(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3));
                moveRelicArm();
                if(gamepad1.left_bumper) {
                    farpid = true;
                }
                if(gamepad1.right_bumper) {
                    closepid = true;
                }
                if(farpid) {
                    turn(FAR_ANGLE, 3.5);
                }
                if(closepid) {
                    turn(CLOSE_ANGLE, 3.5);
                }
            }
            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            if(glyphMode) {
                grabGlyph();
                flip();
                if((Math.abs(gamepad2.right_stick_y) > 0.1)) {
                    lift.setPower(gamepad2.right_stick_y);
                    liftpid = false;
                }
                else {
                    liftpid = true;
                    if(gamepad2.a) {
                        desired_lift_pos = LEVEL_ONE;
                    }
                    else if(gamepad2.x) {
                        desired_lift_pos = LEVEL_TWO;
                    }
                    else if(gamepad2.y) {
                        desired_lift_pos = LEVEL_THREE;
                    }
                    else {
                        desired_lift_pos = lift.getCurrentPosition();
                    }
                    if(liftpid) {
                        lift.setPower(moveLift3(desired_lift_pos));
                    }
                }
            }
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

    public void moveLift() {
        if (gamepad2.right_stick_y < 0) {
            desired_lift_val = desired_lift_val - 1000; //Always keep running at power when joystick is moved
            desired_lift_p = gamepad2.right_stick_y;
            switch1 = true;
            switch2 = false;
            switch3 = true;
        } else if (gamepad2.right_stick_y > 0) {
            desired_lift_val = desired_lift_val + 1000;
            desired_lift_p = gamepad2.right_stick_y;
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
                desired_lift_val = zero_encoder + LEVEL_ONE;
                desired_lift_p = .7;
                switch1 = false; //stop making lift stay at its current position
                switch2 = true;
                t0 = ctime.milliseconds(); //mark time
            }
            if (gamepad2.y) {
                zero();
                desired_lift_val = zero_encoder + LEVEL_THREE;
                desired_lift_p = .7;
                switch1 = false;
                switch2 = true;
                t0 = ctime.milliseconds();
            }
            if (gamepad2.x) {
                zero();
                desired_lift_val = zero_encoder + LEVEL_TWO;
                desired_lift_p = .7;
                switch1 = false;
                switch2 = true;
                t0 = ctime.milliseconds();
            }
            if (switch2) {
                if (ctime.milliseconds() < t0 + 1100) {//account for time needed for lift to get to preset value
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
        lift.setPower(desired_lift_p);
        lift.setTargetPosition((int) desired_lift_val);
    }

    public void moveLift2() {
        if(Math.abs(gamepad2.right_stick_y) > 0.1) {
            lift.setPower(gamepad2.right_stick_y);
            if(gamepad2.right_stick_y > 0) {
                position += 1000;
                lift.setTargetPosition(position);
            }
            else {
                position -= 1000;
                lift.setTargetPosition(position);
            }
        }
        else {
            if(gamepad2.a) {
                switch1 = true;
                if(ctime.milliseconds() < t0 + 1100) {
                    t0 = ctime.milliseconds();
                    lift.setPower(0.7);
                    lift.setTargetPosition((int) LEVEL_ONE);
                    if(lift.getCurrentPosition() == LEVEL_ONE) {
                        switch1 = false;
                    }
                }
            }
            else if(gamepad2.x) {
                switch1 = true;
                if(ctime.milliseconds() < t0 + 1100) {
                    t0 = ctime.milliseconds();
                    lift.setPower(0.7);
                    lift.setTargetPosition((int) LEVEL_TWO);
                    if(lift.getCurrentPosition() == LEVEL_TWO) {
                        switch1 = false;
                    }
                }
            }
            else if(gamepad2.y) {
                switch1 = true;
                if(ctime.milliseconds() < t0 + 1100) {
                    t0 = ctime.milliseconds();
                    lift.setPower(0.7);
                    lift.setTargetPosition((int) LEVEL_THREE);
                    if(lift.getCurrentPosition() == LEVEL_THREE) {
                        switch1 = false;
                    }
                }
            }
            if(!switch1) {
                lift.setPower(0.7);
                lift.setTargetPosition(lift.getCurrentPosition());
            }
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

    public void moveRelicArm() {
        if (gamepad1.y) {
            lig.setPosition(LIG_HALF_STOW);
        }
        if (gamepad1.a) {
            lig.setPosition(LIG_GRAB);
        }
        if (gamepad1.x && (ctime.milliseconds() > (t3+250))) {
            t3 = ctime.milliseconds();
            if (clawPos == CLAW_GRAB) {
                clawPos = CLAW_OPEN;
            }
            else {
                clawPos = CLAW_GRAB;
            }
        }
        claw.setPosition(clawPos);
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

    public void startDegreeController(){
        pT = runtime.time();
        pE = 0;
        tE = 0;
    }

    public void turn(double degree, double margin) {
        if ((Math.abs(getDifference(lastAngles.firstAngle, degree)) > margin ||
                Math.abs(pYaw - lastAngles.firstAngle) > .05)) {
            double change = degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            if (getDifference(lastAngles.firstAngle, degree) > 0) {
                mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, -forwardPower, 0.5);
            } else {
                mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, -backPower, 0.5);
            }
            pYaw = lastAngles.firstAngle;
            resetAngles();
        }
        else {
            mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, 0.5);
            farpid = false;
            closepid = false;
        }
    }

    public double degreeController(double degree){
        double ans = 0;
        double e = Math.abs(getDifference(lastAngles.firstAngle, degree));
        double dE = e-pE;
        double dT = runtime.time() - pT;
        Log.i("PID turn", "e: " + e);
        Log.i("PID Turn", "i: " + i_turn * tE);
        Log.i("PID Turn", "d: " + d_turn * dE / dT);
        Log.i("PID Turn", "p: " + p_turn * e);
        ans = p_turn*e+ i_turn*tE + d_turn*dE/dT;//+f_turn* Math.signum(e);
        pT = runtime.time();
        pE = e;
        tE += e*dT;
        tE = Range.clip(tE * i_turn, -.15, 0.15);///i_turn;
        ans = Range.clip(ans, 0, .7);
        return ans;
    }

    public double liftController(double position) {
        double ans = 0;
        double error = position - lift.getCurrentPosition();
        double p = 0.045, i = 0.002, d = 0.002;
        double dE = error - oldE;
        double dT = runtime.time() - oldT;
        ans = p*error + i*oldtE + d*dE/dT;
        oldT = runtime.time();
        oldE = error;
        oldtE += error*dT;
        oldtE = Range.clip(oldtE * i, -.15, 0.15);
        ans = Range.clip(ans, 0, 1);
        return ans;
    }

    public double moveLift3(double position) {
        double power = 0;
        if(lift.getCurrentPosition() - position > 0) {
            power = -liftController(position);
        }
        else {
            power = liftController(position);
        }
        return power;
    }

    public double getDifference(double beg, double end){
        if (end > beg){
            if (Math.abs(end - beg) < Math.abs((end - 360) - beg)){
                return end - beg;
            } else{
                return  (end-360)-beg;
            }
        } else if(end <= beg){
            if (Math.abs(end - beg) < Math.abs((end + 360) - beg)){
                return end-beg;
            } else{
                return (end+360)-beg;
            }
        }
        return 0;
    }

    public void resetAngles() {
        lastAngles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}