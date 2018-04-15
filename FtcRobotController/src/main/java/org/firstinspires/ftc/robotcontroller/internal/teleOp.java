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
    Servo rflip, lflip, stopper, extendstopper, intakestopper, bottomgrab, topgrab;
    Servo lig, claw;

    BNO055IMU imu;
    Orientation lastAngles;

    final static double CAT_STOW = 0.619444444444444444445;
    final static double KNOCK_CENTER = 0.60000000000000001;

    final static double BOTTOMGRAB_GRAB = 0.0;
    final static double BOTTOMGRAB_STOW = .215;
    final static double TOPGRAB_GRAB = 0.0;
    final static double TOPGRAB_STOW = .215;
    final static double STOPPER_STOP = 0.5;
    final static double STOPPER_STOW = 0.0;
    final static double EXTENDSTOPPER_STOW = 0;
    final static double EXTENDSTOPPER_STOP = 0.5;
    final static double INTAKESTOPPER_STOW = 0.65;
    final static double INTAKESTOPPER_STOP = 0;
    final static double RFLIP_DEPOSIT = 0.159444444444444444;//new grab positions were tested so that the edge of the flipper towards the intake was in line with the top edge of the ramp
    final static double RFLIP_ZERO = 0.679444444444444444445;
    final static double RFLIP_GRAB = 0.759444444444444446;//0.679444444444444444445;
    final static double LFLIP_DEPOSIT = 0.859444444444444445;
    final static double LFLIP_ZERO = 0.269444444444444444443;
    final static double LFLIP_GRAB = 0.209444444444444445;//0.299444444444444444446;

    final static double LIG_STOW = .9094444444444444444;
    final static double LIG_GRAB = .0494444444444444444;//.899444444444444444445;
    final static double LIG_HALF_STOW = .7294444444444444444;
    final static double CLAW_STOW = .209444444444444444;
    final static double CLAW_OPEN = .779444444444444444;
    final static double CLAW_GRAB = .209444444444444444;

    final static double FAR_ANGLE = 90;
    final static double CLOSE_ANGLE = 0;

    final static double LEVEL_ONE = 0;
    final static double LEVEL_TWO = -1140;
    final static double LEVEL_THREE = -1650;

    double changeModeT = 0, changeMultiT = 0, changeClawT = 0, changeStopperT = 0,
            changePreFlipGrabberT = 0, changePreFlipModeT = 0, changeIntakeModeT = 0;
    double intakep = 0;
    double multiplier = 1.0;
    double clawPos = CLAW_STOW;
    boolean glyphMode = true;
    boolean farpid = false, closepid = false;
    boolean preflip = true, preflipgrab = false, zero = false, deposit = false;
    boolean autointake = false;

    double desired_stopper_pos, desired_extendstopper_pos;
    double desired_stopper_delay;
    double desired_lift_pos;
    boolean moveliftpreset;
    boolean delayextendstopper = false, delaystopper = false;

    double p_turn = .045;//0.008;
    double i_turn = .002; //.0045; //.003;
    double d_turn = .002; //.04 //.0045;
    double pT = 0;
    double pE = 0;
    double tE = 0;
    double pYaw = 0;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime sleeptime = new ElapsedTime();
    ElapsedTime stoppertime = new ElapsedTime();

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
        extendstopper = hardwareMap.servo.get("extendstopper");
        intakestopper = hardwareMap.servo.get("intakestopper");
        bottomgrab = hardwareMap.servo.get("bottomgrab");
        topgrab = hardwareMap.servo.get("topgrab");

        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");

        lig = hardwareMap.servo.get("lig");
        claw = hardwareMap.servo.get("relicGrab");

        initialize();
        waitForStart();

        while(opModeIsActive()) {
            cat.setPosition(CAT_STOW);
            knock.setPosition(KNOCK_CENTER);

            if (gamepad1.right_bumper && (sleeptime.milliseconds() > (changeModeT+250))) {
                changeModeT = sleeptime.milliseconds();
                glyphMode = !glyphMode;
            }
            //-----------------------------------------------------------------------------
            // DRIVE ROBOT
            if(glyphMode) {
                multiplier = -Range.clip(gamepad1.left_trigger - 1, -1, -0.4);
            }
            else {
                multiplier = 1.0;
                if (gamepad1.left_bumper && (sleeptime.milliseconds() > (changeMultiT+250))) {
                    changeMultiT = sleeptime.milliseconds();
                    if (multiplier == 1.0) {
                        multiplier = 0.3;
                    }
                    else {
                        multiplier = 1.0;
                    }
                }
            }
            if(!(farpid || closepid)) {
                mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, multiplier);
            }
            if(gamepad1.right_stick_button) {
                closepid = true;
                farpid = false;
            }
            if(gamepad1.left_stick_button) {
                farpid = true;
                closepid = false;
            }
            if(farpid) {
                turn(FAR_ANGLE, 3.5);
            }
            if(closepid) {
                turn(CLOSE_ANGLE, 3.5);
            }
            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT
            if(!glyphMode) {
                relicLift.setPower(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3));
                moveRelicArm();
            }
            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            if(glyphMode) {
                grabGlyph();
                flip();
                moveLift();
                moveStopper();
            }
            //-----------------------------------------------------------------------------
            // TELEMETRY
            telemetry.addData("Lift Encoder Count: ", lift.getCurrentPosition());
            telemetry.addData("Lift Power", lift.getPower());
            telemetry.addData("Intake power", intakep);
            telemetry.addData("Glyph mode", glyphMode);
            telemetry.update();
            //-----------------------------------------------------------------------------
        }
    }

    public void grabGlyph() {
        if(gamepad2.right_bumper || gamepad2.left_bumper || (gamepad1.right_trigger != 0) ||
                (gamepad2.right_trigger != 0) || (gamepad2.left_trigger != 0)) {
            if (gamepad2.right_bumper) {
                intakep = 1.0;
                intakestopper.setPosition(INTAKESTOPPER_STOP);
            }
            if (gamepad2.left_bumper) {
                intakep = .0;
            }
            if (gamepad1.right_trigger > 0.1) {
                runIntake(-1.0 * Math.signum(gamepad1.right_trigger)
                        , -1.0 * Math.signum(gamepad1.right_trigger));
            } else if (gamepad2.right_trigger > 0.85 || gamepad2.left_trigger > 0.85) {
                runIntake(-1.0 * Math.signum(gamepad2.right_trigger)
                        , -1.0 * Math.signum(gamepad2.left_trigger));
            } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                runIntake(-0.5 * Math.signum(gamepad2.right_trigger)
                        , -0.5 * Math.signum(gamepad2.left_trigger));
            } else {
                runIntake(intakep, intakep);
            }
            autointake = true;
        }
        else {
            runAutoIntake();
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
        if (gamepad2.b) {
            preFlipGrab();
        }
        if ((gamepad1.b && sleeptime.milliseconds() > changePreFlipGrabberT + 250)) {
            changePreFlipGrabberT = sleeptime.milliseconds();
            preflipgrab = !preflipgrab;
            if (preflipgrab) {
                preFlipGrab();
            } else {
                preFlipStow();
            }
        }
        if (gamepad1.left_bumper && sleeptime.milliseconds() > changePreFlipModeT + 250) { //toggle to switch between PreFlip and Regular Flipper
            changePreFlipModeT = sleeptime.milliseconds();
            preflip = !preflip;
        }
    }

    public void runIntake(double rpower, double lpower) {
        rintake.setPower(rpower);
        lintake.setPower(lpower);
        if(rpower < 0 && lpower < 0) {
            intakestopper.setPosition(INTAKESTOPPER_STOW);
        }
        else {
            intakestopper.setPosition(INTAKESTOPPER_STOP);
        }
    }

    public void runAutoIntake() {
        if(autointake) {
            if ((lift.getCurrentPosition() > LEVEL_ONE) || deposit) {
                runIntake(-1.0, -1.0);
                intakestopper.setPosition(INTAKESTOPPER_STOW);
            } else {
                runIntake(1.0, 1.0);
                intakestopper.setPosition(INTAKESTOPPER_STOP);
            }
        }
    }

    public void grab() {
        preFlipStow();
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
        //  stopperStopWithDelay();
        if (zero) {
            stopper.setPosition(STOPPER_STOP);
            zero = false;
        } else {
            stopperStopWithDelay();
        }
    }
    public void zero() {
        zero = true;
        preFlipGrab();
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
        if (Math.abs(extendstopper.getPosition() - EXTENDSTOPPER_STOP) < 0.03) {//desired_extendstopper_pos == EXTENDSTOPPER_STOP
            stopper.setPosition(STOPPER_STOW);
        } else {
            extendstopper.setPosition(EXTENDSTOPPER_STOP);
        }
    }
    public void deposit() {
        // rflip.setPosition(RFLIP_DEPOSIT);
        // lflip.setPosition(LFLIP_DEPOSIT);
        deposit = true;
        preflipgrab = true;
        preFlipGrab();
        // stopperStowWithDelay();
        if (zero) {
            extendstopper.setPosition(EXTENDSTOPPER_STOW);
            rflip.setPosition(RFLIP_DEPOSIT); //Deposits only after both the stopper and extendstopper's been stown away
            lflip.setPosition(LFLIP_DEPOSIT);
            if (!preflip) {
                topgrab.setPosition(TOPGRAB_STOW);
            }
            zero = false;
        } else {
            stopperStowWithDelay();
        }
    }

    public void preFlipGrab() {
        if (preflip) {
            bottomgrab.setPosition(BOTTOMGRAB_GRAB);
            topgrab.setPosition(TOPGRAB_GRAB);
        } else {
            bottomgrab.setPosition(BOTTOMGRAB_STOW);
            topgrab.setPosition(TOPGRAB_GRAB);
        }
    }
    public void preFlipStow() {
        bottomgrab.setPosition(BOTTOMGRAB_STOW);
        topgrab.setPosition(TOPGRAB_STOW);
    }

    public void moveRelicArm() {
        if (gamepad1.y) {
            lig.setPosition(LIG_HALF_STOW);
        }
        if (gamepad1.a) {
            lig.setPosition(LIG_GRAB);
        }
        if (gamepad1.x && (sleeptime.milliseconds() > (changeClawT+250))) {
            changeClawT = sleeptime.milliseconds();
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
        double v1 = vd*Math.sin(theta + (Math.PI/4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))+v0; //fr
        double v3 = vd*Math.cos(theta + (Math.PI/4))-v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));
        if((v1 > 0 && v2 > 0 && v3 > 0 && v4 > 0) || (v1 < 0 && v2 < 0 && v3 < 0 && v4 < 0)) {
            if(multiplier < 0.5) {
                multiplier = 0.5;
            }
        }
        if (max > 1) {
            fl.setPower(multiplier * v1/max);
            fr.setPower(multiplier * v2/max);
            bl.setPower(multiplier * v3/max);
            br.setPower(multiplier * v4/max);
        } else {
            fl.setPower(multiplier * v1);
            fr.setPower(-multiplier * v2);
            bl.setPower(-multiplier * v3);
            br.setPower(multiplier * v4);
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

    public void stopperStopWithDelay() {
        changeStopperT = stoppertime.milliseconds();
        delaystopper = true;
        delayextendstopper = false;
        desired_stopper_delay = 300;
        desired_stopper_pos = STOPPER_STOP;
        desired_extendstopper_pos = EXTENDSTOPPER_STOP;
    }
    public void stopperStowWithDelay() {
        changeStopperT = stoppertime.milliseconds();
        delaystopper = false;
        delayextendstopper = true;
        desired_stopper_delay = 300;
        desired_stopper_pos = STOPPER_STOW;
        desired_extendstopper_pos = EXTENDSTOPPER_STOW;
    }

    public void moveStopper() {
        if (delayextendstopper) {
            stopper.setPosition(desired_stopper_pos);
            if (sleeptime.milliseconds() > changeStopperT + desired_stopper_delay) {
                changeStopperT = sleeptime.milliseconds();
                extendstopper.setPosition(desired_extendstopper_pos);
                if (deposit) { //Deposit boolean is here because we want to have the stopper move but the flipper stay where it is.
                    rflip.setPosition(RFLIP_DEPOSIT); //Deposits only after both the stopper and extendstopper's been stown away
                    lflip.setPosition(LFLIP_DEPOSIT);
                    if (!preflip) {
                        bottomgrab.setPosition(BOTTOMGRAB_STOW);
                    }
                    deposit = false;
                }
                delayextendstopper = false;
            }
        }
        if (delaystopper) {
            extendstopper.setPosition(desired_extendstopper_pos);
            if (sleeptime.milliseconds() > changeStopperT + desired_stopper_delay) {
                changeStopperT = sleeptime.milliseconds();
                stopper.setPosition(desired_stopper_pos);
                delaystopper = false;
            }
        }
    }

    public void moveLift() {
        if(gamepad2.right_stick_y > 0 ) {
            lift.setPower(Range.clip(gamepad2.right_stick_y, .1, 1));
            moveliftpreset = false;
        } else if (gamepad2.right_stick_y < 0) {
            lift.setPower(Range.clip(gamepad2.right_stick_y, -1, -.1));
            moveliftpreset = false;
        }
        else {
            if(gamepad2.a) {
                desired_lift_pos = LEVEL_ONE;
                moveliftpreset = true;
            }
            else if(gamepad2.x) {
                desired_lift_pos = LEVEL_TWO;
                stopper.setPosition(STOPPER_STOW);
                moveliftpreset = true;
            }
            else if(gamepad2.y) {
                desired_lift_pos = LEVEL_THREE;
                stopper.setPosition(STOPPER_STOW);
                moveliftpreset = true;
            }
            if (moveliftpreset) {
                if (Math.abs(lift.getCurrentPosition() - desired_lift_pos) < 30) {
                    lift.setPower(0.0);
                    moveliftpreset = false;
                }
                else {
                    lift.setPower(Math.signum(lift.getCurrentPosition()-desired_lift_pos)*-.7); //May need to be reversed
                }
            } else {
                lift.setPower(.0);
            }

        }
    }

    public void initialize() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //so that lift can hold its position
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
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
        knock.setPosition(KNOCK_CENTER);
        lig.setPosition(LIG_HALF_STOW);
        claw.setPosition(CLAW_STOW);
        stopper.setPosition(STOPPER_STOW);
        extendstopper.setPosition(EXTENDSTOPPER_STOP);

        startDegreeController();
    }
}