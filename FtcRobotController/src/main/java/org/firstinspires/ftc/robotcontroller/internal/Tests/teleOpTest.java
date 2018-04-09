package org.firstinspires.ftc.robotcontroller.internal.Tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by sahith on 11/10/17.
 */
public class teleOpTest extends LinearOpMode {
    DcMotor frdrive, fldrive, brdrive, bldrive;
    DcMotor rintake, lintake;
    DcMotor lift;
    Servo rflip, lflip;
    Servo stopper, extendstopper;
    Servo bottomgrab, topgrab;
    final static double BOTTOMGRAB_GRAB = 0;
    final static double BOTTOMGRAB_STOW = .215;
    final static double TOPGRAB_GRAB = 0.06;
    final static double TOPGRAB_STOW = .209;
    final static double STOPPER_STOP = 0.5;
    final static double STOPPER_STOW = 0.0;
    final static double EXTENDSTOPPER_STOW = 0;
    final static double EXTENDSTOPPER_STOP = 0.5;
    final static double RFLIP_DEPOSIT = 0.159444444444444444;//new grab positions were tested so that the edge of the flipper towards the intake was in line with the top edge of the ramp
    final static double RFLIP_ZERO = 0.679444444444444444445;
    final static double RFLIP_GRAB = 0.759444444444444446;//0.679444444444444444445;
    final static double LFLIP_DEPOSIT = 0.859444444444444445;
    final static double LFLIP_ZERO = 0.269444444444444444443;
    final static double LFLIP_GRAB = 0.209444444444444445;//0.299444444444444444446;
    final static double LIFT_INIT = 0;
    final static double LIFT_TWO = 1140;
    final static double LIFT_THREE = 1650;
    double tdeposit, tgrab, tpreflipmode;
    double intakep;
    double multiplier;
    double desired_stopper_pos, desired_extendstopper_pos;
    double desired_stopper_delay;
    double desired_lift_pos;
    boolean moveliftpreset;
    boolean delayextendstopper, delaystopper;
    boolean preflipgrab;
    boolean deposit;
    boolean preflip, zero;
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
        extendstopper = hardwareMap.servo.get("extendstopper");
        bottomgrab = hardwareMap.servo.get("bottomgrab");
        topgrab = hardwareMap.servo.get("topgrab");
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
        delayextendstopper = false;
        delaystopper = false;
        preflipgrab = false;
        deposit = false;
        preflip = true;
        zero = false;
        desired_stopper_pos = STOPPER_STOW;
        desired_extendstopper_pos = EXTENDSTOPPER_STOW;
        stopper.setPosition(desired_stopper_pos);
        extendstopper.setPosition(desired_extendstopper_pos);
        preFlipStow();
        fldrive.setPower(0);
        frdrive.setPower(0);
        bldrive.setPower(0);
        brdrive.setPower(0);
        lift.setPower(0);
        rintake.setPower(0);
        lintake.setPower(0);
        multiplier = 1.0;
        intakep = .0;
        tdeposit = 0;
        tgrab = 0;
        desired_lift_pos = LIFT_INIT;
        waitForStart();
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
        stopperStopWithDelay();
        while(opModeIsActive()) {
            //-----------------------------------------------------------------------------
            // DRIVE ROBOT
            multiplier = -Range.clip(gamepad1.left_trigger - 1, -1, -0.3);
            mecanum(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, multiplier);
            grabGlyph();
            flip();
            moveLift4();
            moveStopper();
            //-----------------------------------------------------------------------------
            // TELEMETRY
            telemetry.addData("Lift Encoder Count: ", lift.getCurrentPosition());
            telemetry.addData("Lift Power: ", lift.getPower());
            telemetry.addData("Intake Power: ", intakep);
            telemetry.addData("Drive Multiplier", multiplier);
            if (preflip) {
                telemetry.addData("Mode: ", "PreFlip");
            } else {
                telemetry.addData("Mode: ", "RegularFlip");
            }
            telemetry.update();
        }
    }
    public void mecanum(double joyly, double joylx, double joyrx, double multiplier) {
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joyly, joylx);
        double v0 = joyrx;
        //v0 = triggers;
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
    public void moveLift4() {
        if(gamepad2.right_stick_y > 0 ) {
            lift.setPower(-Range.clip(gamepad2.right_stick_y, .1, 1));
            moveliftpreset = false;
        } else if (gamepad2.right_stick_y < 0) {
            lift.setPower(-Range.clip(gamepad2.right_stick_y, -1, -.1));
            moveliftpreset = false;
        }
        else {
            if(gamepad2.a) {
                desired_lift_pos = LIFT_INIT;
                moveliftpreset = true;
            }
            else if(gamepad2.x) {
                desired_lift_pos = LIFT_TWO;
                stopper.setPosition(STOPPER_STOW);
                moveliftpreset = true;
            }
            else if(gamepad2.y) {
                desired_lift_pos = LIFT_THREE;
                stopper.setPosition(STOPPER_STOW);
                moveliftpreset = true;
            }
            if (moveliftpreset) {
                if (Math.abs(lift.getCurrentPosition() - desired_lift_pos) < 30) {
                    lift.setPower(0.0);
                    moveliftpreset = false;
                }
                else {
                    lift.setPower(-Math.signum(lift.getCurrentPosition()-desired_lift_pos)); //May need to be reversed
                }
            } else {
                lift.setPower(.0);
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
        if (gamepad1.b && ctime.milliseconds() > tgrab + 250) {
            tgrab = ctime.milliseconds();
            preflipgrab = !preflipgrab;
            if (preflipgrab) {
                preFlipGrab();
            } else {
                preFlipStow();
            }
        }
        if (gamepad1.left_bumper && ctime.milliseconds() > tpreflipmode + 250) { //toggle to switch between PreFlip and Regular Flipper
            preflip = !preflip;
            tpreflipmode = ctime.milliseconds();
        }
    }
    public void runIntake(double rpower, double lpower) {
        rintake.setPower(rpower);
        lintake.setPower(lpower);
    }
    //At grab, the extendstopper and stopper are in stopping form
    //At zero, the stopper is stowed and the extendstopper is in stopping form
    //At deposit, the extendstopper and stopper are stowed
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
            zero = false;
        } else {
            stopperStowWithDelay();
        }
    }
    public void stopperStopWithDelay() {
        tdeposit = ctime.milliseconds();
        delaystopper = true;
        delayextendstopper = false;
        desired_stopper_delay = 250;
        desired_stopper_pos = STOPPER_STOP;
        desired_extendstopper_pos = EXTENDSTOPPER_STOP;
    }
    public void stopperStowWithDelay() {
        tdeposit = ctime.milliseconds();
        delaystopper = false;
        delayextendstopper = true;
        desired_stopper_delay = 250;
        desired_stopper_pos = STOPPER_STOW;
        desired_extendstopper_pos = EXTENDSTOPPER_STOW;
    }
    public void moveStopper() {
        if (delayextendstopper) {
            stopper.setPosition(desired_stopper_pos);
            if (ctime.milliseconds() > tdeposit + desired_stopper_delay) {
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
            if (ctime.milliseconds() > tdeposit + desired_stopper_delay) {
                stopper.setPosition(desired_stopper_pos);
                delaystopper = false;
            }
        }
    }
    public void preFlipGrab() {
        if (preflip) {
            bottomgrab.setPosition(BOTTOMGRAB_GRAB);
            topgrab.setPosition(TOPGRAB_GRAB);
        } else {
            bottomgrab.setPosition(BOTTOMGRAB_GRAB);
            topgrab.setPosition(TOPGRAB_STOW);
        }
    }
    public void preFlipStow() {
        bottomgrab.setPosition(BOTTOMGRAB_STOW);
        topgrab.setPosition(TOPGRAB_STOW);
    }
}