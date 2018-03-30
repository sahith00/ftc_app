package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by vulcanrobotics8375 on 1/31/18.
 */

public class liftTest extends LinearOpMode {
    DcMotor lift;

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

    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //so that lift can hold its position
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

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
        while (opModeIsActive()) {
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
                desired_lift_val = zero_encoder + LEVEL_THREE;
                desired_lift_p = .7;
                switch1 = false;
                switch2 = true;
                t0 = ctime.milliseconds();
            }
            if (gamepad2.x) {
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
}