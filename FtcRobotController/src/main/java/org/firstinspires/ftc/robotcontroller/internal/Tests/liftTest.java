package org.firstinspires.ftc.robotcontroller.internal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by vulcanrobotics8375 on 1/31/18.
 */

public class liftTest extends LinearOpMode {
    DcMotor lift;
    boolean switch1, switch2, switch3;
    double zero_encoder, t;
    double desired_lift_val, desired_lift_power, currentshit;
    final double SECOND_ROW = -500, THIRD_ROW = -1000; //random values right now
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(.0);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        zero_encoder = 0;
        desired_lift_val = 0;
        desired_lift_power = 0;
        currentshit = 0;
        switch1 = true;
        switch2 = false;
        switch3 = false;
        t = 0;
        ElapsedTime ctime = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
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
                    currentshit = lift.getCurrentPosition();
                    switch3 = false;
                }
                if (switch1) { //if we aren't moving our lift at all, have it stay at its last current position
                    //   desired_lift_power = .0;
                    desired_lift_val = currentshit;
                }
                if (gamepad2.a) {
                    desired_lift_power = .7;
                    desired_lift_val = zero_encoder;
                    switch1 = false; //stop making lift stay at its current position
                    switch2 = true;
                    t = ctime.milliseconds(); //mark time
                }
                if (gamepad2.x) {
                    desired_lift_power = .7;
                    desired_lift_val = zero_encoder + SECOND_ROW;;
                    switch1 = false;
                    switch2 = true;
                    t = ctime.milliseconds();
                }
                if (gamepad2.y) {
                    desired_lift_power = .7;
                    desired_lift_val = zero_encoder + THIRD_ROW;
                    switch1 = false;
                    switch2 = true;
                    t = ctime.milliseconds();
                }
                if (switch2) {
                    if (ctime.milliseconds() < t + 1100) {//account for time needed for lift to get to preset value
                        switch1 = false;
                    } else {
                        switch1 = true;
                        switch3 = true;
                        switch2 = false;
                    }
                }
                if (gamepad2.b) {
                    zero_encoder = lift.getCurrentPosition();
                }
            }
            lift.setPower(desired_lift_power);
            lift.setTargetPosition((int) desired_lift_val);
        }
    }
}