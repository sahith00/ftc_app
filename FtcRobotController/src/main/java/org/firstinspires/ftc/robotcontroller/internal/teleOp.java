package org.firstinspires.ftc.robotcontroller.internal;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Drivetrain;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Flipper;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Intake;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Jewel;
import org.firstinspires.ftc.robotcontroller.internal.Subsystems.Lift;

/**
 * Created by sahith on 12/10/17.
 */
public class teleOp extends LinearOpMode{
    Drivetrain drivetrain;
    Flipper flipper;
    Intake intake;
    Jewel jewel;
    Lift lift;
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
        drivetrain = new Drivetrain(hardwareMap);
        flipper = new Flipper(hardwareMap);
        intake = new Intake(hardwareMap);
        jewel = new Jewel(hardwareMap);
        lift = new Lift(hardwareMap);
        drivetrain.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.fr.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //so that lift can hold its position
        lift.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        flipper.grab(); //inits flipper to grab position to prepare for grabbing
        drivetrain.fl.setPower(0);
        drivetrain.fr.setPower(0);
        drivetrain.bl.setPower(0);
        drivetrain.br.setPower(0);
        lift.lift.setPower(0);
        intake.rintake.setPower(0);
        intake.lintake.setPower(0);
        jewel.stowCat();
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
            jewel.stowCat();
            //-----------------------------------------------------------------------------
            // DRIVE ROBOT

            drivetrain.mecanum(gamepad1.left_stick_y, -(gamepad1.left_stick_x), -(gamepad1.right_stick_x), multiplier);
            //-----------------------------------------------------------------------------
            // GRAB RELIC AND DEPOSIT
            //-----------------------------------------------------------------------------
            // GRAB GLYPH AND DEPOSIT
            grabGlyph();
            moveLift();
            flip();
            //-----------------------------------------------------------------------------
            // TELEMETRY
            telemetry.addData("Lift Encoder Count: ", lift.lift.getCurrentPosition());
            telemetry.addData("Lift Power", lift.lift.getPower());
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
            intake.runIntake(-0.65 * Math.signum(gamepad2.right_trigger), -0.65 * Math.signum(gamepad2.left_trigger));
        }
        else {
            intake.runIntake(intakep, intakep);
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
                currentpos = lift.getLiftPosition();
                switch3 = false;
            }
            if (switch1) { //if we aren't moving our lift at all, have it stay at its last current position
                //   desired_lift_power = .0;
                desired_lift_val = currentpos;
            }
            if (gamepad2.a) {
                flipper.grab();
                desired_lift_power = .7;
                desired_lift_val = zero_encoder + LEVEL_ONE;
                switch1 = false; //stop making lift stay at its current position
                switch2 = true;
                t = ctime.milliseconds(); //mark time
            }
            if (gamepad2.y) {
                flipper.zero();
                desired_lift_power = .7;
                desired_lift_val = zero_encoder + LEVEL_THREE;;
                switch1 = false;
                switch2 = true;
                t = ctime.milliseconds();
            }
            if (gamepad2.x) {
                flipper.zero();
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
                zero_encoder = lift.getLiftPosition();
            }
        }
        lift.runLift(desired_lift_power, (int) desired_lift_val);
    }

    public void flip() {
        if (gamepad1.x) { //zero position, when flipper is parallel to ground
            flipper.zero();
        }
        if (gamepad1.a) { //grab position, when we're picking up cubes
            flipper.grab();
        }
        if (gamepad1.y) { //deposit position, when we're depositing cubes
            flipper.deposit();
        }
        if (gamepad2.dpad_left) { //zero position, when flipper is parallel to ground
            flipper.zero();
        }
        if (gamepad2.dpad_down) { //grab position, when we're picking up cubes
            flipper.grab();
        }
        if (gamepad2.dpad_up) { //deposit position, when we're depositing cubes
            flipper.deposit();
        }
    }
}