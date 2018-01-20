package org.firstinspires.ftc.robotcontroller.internal.Cisco;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sahith on 12/10/17.
 */
public class ciscoTeleOp extends LinearOpMode{
    DcMotor frdrive, fldrive, brdrive, bldrive;
    DcMotor rgrab, lgrab;
    DcMotor lift;
    //    Servo cat, knock;
    Servo rflip, lflip;
    final static double CAT_STOW = 0.66;
    final static double KNOCK_CENTER = 0.38;
    final static double RFLIP_DEPOSIT = 0.215; //0.47
    final static double RFLIP_ZERO = 0.505; //0.74
    final static double RFLIP_GRAB = 0.575;
    final static double LFLIP_DEPOSIT = 0.87995; //0.53
    final static double LFLIP_ZERO = 0.3495; //0.26
    final static double LFLIP_GRAB = 0.2395;
    final static double LEVEL_ONE = 0;
    final static double LEVEL_TWO = -372;
    final static double LEVEL_THREE = -770;
    double intakep;
    boolean lift_zero;
    double multiplier = 1.0;

    double p_turn = .03;//0.008;
    double i_turn = .00; //.0045; //.003;
    double d_turn = .002; //.04 //.0045;
    double f_turn = .0;
    double pT = 0;
    double pE = 0;
    double tE = 0;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fldrive = hardwareMap.dcMotor.get("fldrive");
        frdrive = hardwareMap.dcMotor.get("frdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        lift = hardwareMap.dcMotor.get("lift");
        rgrab = hardwareMap.dcMotor.get("rintake");
        lgrab = hardwareMap.dcMotor.get("lintake");
//        cat = hardwareMap.servo.get("cat");
//        knock = hardwareMap.servo.get("knock");
        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");
        fldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fldrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bldrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //so that lift can hold its position
        grab(); //inits flipper to grab position to prepare for grabbing
        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        lift.setPower(0);
        rgrab.setPower(0);
        lgrab.setPower(0);
/*        cat.setPosition(CAT_STOW);
        knock.setPosition(KNOCK_CENTER);*/
        intakep = .0;
        lift_zero = true;

        waitForStart();
        while(opModeIsActive()) {
          /*  cat.setPosition(CAT_STOW);
            knock.setPosition(KNOCK_CENTER);*/
            // DRIVE ROBOT
            if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down) {
                if (gamepad1.dpad_up) {
                    fldrive.setPower(.8);
                    frdrive.setPower(.8);
                    bldrive.setPower(.8);
                    brdrive.setPower(.8);
                }
                if (gamepad1.dpad_right) {
                    fldrive.setPower(.8);
                    frdrive.setPower(-.8);
                    bldrive.setPower(-.8);
                    brdrive.setPower(.8);
                }
                if (gamepad1.dpad_left) {
                    fldrive.setPower(-.8);
                    frdrive.setPower(.8);
                    bldrive.setPower(.8);
                    brdrive.setPower(-.8);
                }
                if (gamepad1.dpad_down) {
                    fldrive.setPower(-.8);
                    frdrive.setPower(-.8);
                    bldrive.setPower(-.8);
                    brdrive.setPower(-.8);
                }
            } else {
                mecanum(fldrive, frdrive, bldrive, brdrive, multiplier);
            }
            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT
            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            grabGlyph();
            liftGlyph();
            useFlipper(gamepad1);         //Driver 1
            useFlipper(gamepad2);         //Driver 2
            //-----------------------------------------------------------------------------
            // TELEMETRY
            telemetry.addData("Lift Encoder Count: ", lift.getCurrentPosition());
            telemetry.addData("Lift Power", lift.getPower());
            telemetry.addData("Intake power", intakep);
            telemetry.update();
            //-----------------------------------------------------------------------------
        }
    }
    public void mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, double multiplier) {
        double joyly = gamepad1.left_stick_y;
        double joylx = -(gamepad1.left_stick_x);
        double joyrx = -(gamepad1.right_stick_x);
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
    public void grab() {
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
    }
    public void zero() {
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
    }
    public void deposit() {
        rflip.setPosition(RFLIP_DEPOSIT);
        lflip.setPosition(LFLIP_DEPOSIT);
    }

    public void movePID(double distance, double margin) {
//        startDegreeController();
//        double pYaw = lift.getCurrentPosition();
//        while (Math.abs(lift.getCurrentPosition() - distance) > margin || Math.abs(pYaw - lift.getCurrentPosition()) > .05) {
//            double change = tickController(distance);
//            lift.setPower(Range.clip(change, -1, 1));
//            pYaw = lift.getCurrentPosition();
//            telemetry.addData("In While loop", lift.getCurrentPosition());
//            if (gamepad2.back) {
//                break;
//            }
//        }
        if (lift.getCurrentPosition() - distance < 0) {
            lift.setPower(0.5);
        }
        else {
            lift.setPower(-0.5);
        }
        while (Math.abs(lift.getCurrentPosition() - distance) > margin ) {
            mecanum(fldrive, frdrive, bldrive, brdrive, multiplier);
            useFlipper(gamepad1);
            useFlipper(gamepad2);
            grabGlyph();
        }
        lift.setPower(0.0);
    }

    public void startDegreeController(){
        pT = runtime.time();
        pE = 0;
        tE = 0;
    }

    public double tickController(double ticks){
        double ans = 0;
        double e = -(lift.getCurrentPosition() - ticks);
        double dE = e-pE;
        double dT = runtime.time() - pT;
        Log.i("PID turn", "e: " + e);
        Log.i("PID Turn", "i: " + i_turn * tE);
        Log.i("PID Turn", "d: " + d_turn * dE / dT);
        Log.i("PID Turn", "p: " + p_turn * e);
        ans = p_turn*e+ i_turn*tE + d_turn*dE/dT;// +f_turn* Math.signum(e);
        pT = runtime.time();
        pE = e;
        tE += e*dT;
//        if (Math.signum(e) != Math.signum(pE)){
//            tE = 0;
//        }
        tE = Range.clip(tE * i_turn, -.15, 0.15);///i_turn;
        ans = Range.clip(ans, 0, .7);
        return ans;
    }

    public void moveLevels(double level) {
        lift_zero = false;
        movePID(level, 30);
        lift_zero = true;
    }

    public void grabGlyph() {
        if (gamepad2.right_bumper) {
            intakep = 1.0;
        }
        if (gamepad2.left_bumper) {
            intakep = .0;
        }
        if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            rgrab.setPower(-0.5 * Math.signum(gamepad2.right_trigger));
            lgrab.setPower(-0.5 * Math.signum(gamepad2.left_trigger));
        }
        else {
            rgrab.setPower(Range.clip(intakep, -1, 1));
            lgrab.setPower(Range.clip(intakep, -1, 1));
        }
    }

    public void useFlipper(Gamepad gamepad) {
        if (gamepad.dpad_left) { //zero position, when flipper is parallel to ground
            zero();
        }
        if (gamepad.dpad_down) { //grab position, when we're picking up cubes
            grab();
        }
        if (gamepad.dpad_up) { //deposit position, when we're depositing cubes
            deposit();
        }
    }

    public void liftGlyph() {
        if (lift_zero) {
            lift.setPower(Range.clip(gamepad2.right_stick_y, -.6, .5));
        }

        if (gamepad2.y) {
            moveLevels(LEVEL_THREE);
        }
        if (gamepad2.x) {
            moveLevels(LEVEL_TWO);
        }
        if (gamepad2.a) {
            moveLevels(LEVEL_ONE);
        }
    }
}