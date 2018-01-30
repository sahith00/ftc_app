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
    Servo cat;
    Servo rflip, lflip, stopper;
    final static double CAT_STOW = 0.79;
    final static double STOPPER_STOP = 0.959444444444444445;
    final static double STOPPER_DEPOSIT = 0.62000000000000001;
    final static double STOPPER_ZERO = 0.16944444444444452;
    final static double RFLIP_DEPOSIT = 0.7794444444444446;
    final static double RFLIP_ZERO = 0.199444444444444444448;
    final static double RFLIP_GRAB = 0.0500000000000000000044;
    final static double LFLIP_DEPOSIT = 0.040000000000000036;
    final static double LFLIP_ZERO = 0.59000000000000000001;
    final static double LFLIP_GRAB = 0.719444444444444444446;
    final static double LEVEL_ONE = 0;
    final static double LEVEL_TWO = -372;
    final static double LEVEL_THREE = -770;
    double intakep;
    boolean lift_zero;
    double multiplier = 1.0;
    double zero_encoder = 0.0;

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
        cat = hardwareMap.servo.get("cat");
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
        grab(); //inits flipper to grab position to prepare for grabbing
        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        lift.setPower(0);
        rgrab.setPower(0);
        lgrab.setPower(0);
        cat.setPosition(CAT_STOW);
        intakep = .0;
        lift_zero = true;

        waitForStart();
        while(opModeIsActive()) {
            cat.setPosition(CAT_STOW);
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
            if (gamepad2.b) {
                zero_encoder = lift.getCurrentPosition();
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

    public void moveLevels(double level) {
        lift_zero = false;
        movePID(zero_encoder + level, 30);
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
            rgrab.setPower(-0.65 * Math.signum(gamepad2.right_trigger));
            lgrab.setPower(-0.65 * Math.signum(gamepad2.left_trigger));
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