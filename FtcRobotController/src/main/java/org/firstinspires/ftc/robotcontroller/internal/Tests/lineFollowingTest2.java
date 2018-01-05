package org.firstinspires.ftc.robotcontroller.internal.Tests;

        import android.util.Log;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.Func;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

        import java.util.Locale;
/**
 * Created by sahith on 1/3/18.
 */

public class lineFollowingTest2 extends LinearOpMode {
    ColorSensor lineSensor;
    DcMotor fr, fl, br, bl;
    int firstBlue;
    int[] firstBlueRange, notBlueRange;
    double firstAngle;
    double kp, ki, kd;
    double error, lasterror, lasttime;
    double desiredblue;
    double dE, dT;
    double pidcalcvalue;
    double left, right;
    double integral;
    double initialp = .1;
    BNO055IMU imu;
    Orientation lastAngles;

    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        lineSensor = hardwareMap.colorSensor.get("lineSensor");
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        setConstants(.005, .0, .05);
        error = 0;
        lasterror = 0;
        lasttime = 0;
        desiredblue = 18;
        dE = 0.0;
        dT = 0.0;
        integral = .0;
        waitForStart();
        while (opModeIsActive()) {
            error = 0.5 * (lineSensor.blue() - desiredblue);
            dE = error - lasterror;
            dT = runtime.time() - lasttime;
            lasterror = error;
            lasttime = runtime.time();
            integral += error*dT;
            pidcalcvalue = kp*error + ki*(integral) + kd*(dE/dT);
            left = Range.clip(initialp + pidcalcvalue, .0, 1.0);
            right = Range.clip(initialp - pidcalcvalue, .0, 1.0);
            setLeft(left, left);
            setRight(right, right);

            telemetry.addData("pid", pidcalcvalue);
            telemetry.addData("fr", fr.getPower());
            telemetry.addData("fl", fl.getPower());
            telemetry.addData("br", br.getPower());
            telemetry.addData("bl", bl.getPower());
            telemetry.update();
        }
    }
    public void setLeft(double fl1, double bl1) {
        fl.setPower(fl1);
        bl.setPower(bl1);
    }
    public void setRight(double fr1, double br1) {
        fr.setPower(fr1);
        br.setPower(br1);
    }
    public void setConstants(double kp, double ki,double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public boolean listIncludesElement(int[] l, int x) {
        for (int i = 0; i < l.length; i++) {
            if (x == l[i]) {
                return true;
            }
        }
        return false;
    }
}
