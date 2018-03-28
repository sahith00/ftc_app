package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sahith on 11/10/17.
 */
public class teleOpTest extends LinearOpMode {
    DcMotor frdrive, fldrive, brdrive, bldrive;
    DcMotor rintake, lintake;
    DcMotor lift;

    Servo rflip, lflip, stopper;

    BNO055IMU imu;
    Orientation lastAngles;

    final static double STOPPER_STOP = 0.0;
    final static double STOPPER_STOW = 0.5;
    final static double RFLIP_DEPOSIT = 0.069444444444444448;
    final static double RFLIP_ZERO = 0.629444444444444444445;
    final static double RFLIP_GRAB = 0.679444444444444444445;
    final static double LFLIP_DEPOSIT = 0.93;
    final static double LFLIP_ZERO = 0.359444444444444444445;
    final static double LFLIP_GRAB = 0.299444444444444444446;

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
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");

        lift = hardwareMap.dcMotor.get("lift");
        rintake = hardwareMap.dcMotor.get("rintake");
        lintake = hardwareMap.dcMotor.get("lintake");

        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");
        stopper = hardwareMap.servo.get("stopper");

        fldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        lift.setPower(0);
        rintake.setPower(0);
        lintake.setPower(0);

        multiplier = 1.0;
        intakep = .0;
        lift_zero = true;
        zero_encoder = 0;
        desired_lift_val = 0;
        currentpos = 0;
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

            if (gamepad1.b && (ctime.milliseconds() > (t1+250))) {
                t1 = ctime.milliseconds();
                glyphMode = !glyphMode;
            }
            //-----------------------------------------------------------------------------
            // DRIVE ROBOT
            multiplier = -Range.clip(gamepad1.left_trigger - 1, -1, -0.3);

            mecanum(gamepad1.left_stick_y, -gamepad1.left_stick_x, -(gamepad1.right_stick_x), multiplier);

            if(glyphMode) {
                grabGlyph();
                flip();
                moveLift4();
            }
            //-----------------------------------------------------------------------------
            // TELEMETRY
            telemetry.addData("Lift Encoder Count: ", lift.getCurrentPosition());
            telemetry.addData("Lift Power", lift.getPower());
            telemetry.addData("Intake power", intakep);
            telemetry.addData("multiplier", multiplier);
            telemetry.update();
        }
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
            fldrive.setPower(multiplier * (v1/max));
            frdrive.setPower(multiplier * (v2/max));
            bldrive.setPower(multiplier * (v3/max));
            brdrive.setPower(multiplier * (v4/max));
        } else {
            fldrive.setPower(multiplier*v1);
            frdrive.setPower(multiplier*v2);
            bldrive.setPower(multiplier*v3);
            brdrive.setPower(multiplier*v4);
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

    public void grabGlyph() {
        if (gamepad2.right_bumper) {
            intakep = 1.0;
        }
        if (gamepad2.left_bumper) {
            intakep = .0;
        }
        if (gamepad2.right_trigger > 0.85 || gamepad2.left_trigger > 0.85) {
            runIntake(-1.0 * Math.signum(gamepad2.right_trigger)
                    , -1.0 * Math.signum(gamepad2.left_trigger));
        }
        if (gamepad1.right_trigger > 0.1) {
            runIntake(-1.0 * Math.signum(gamepad1.right_trigger)
                    , -1.0 * Math.signum(gamepad1.right_trigger));
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

    public void moveLift4() {
        if((Math.abs(gamepad2.right_stick_y) > 0.1)) {
            lift.setPower(gamepad2.right_stick_y);
        }
        else {
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
            if (Math.abs(lift.getCurrentPosition() - desired_lift_pos) < 0.1) {
                lift.setPower(0.0);
            }
            else {
                if(lift.getCurrentPosition() > desired_lift_pos) {
                    lift.setPower(-0.7);
                }
                else {
                    lift.setPower(0.7);
                }
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
}
