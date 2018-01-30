package org.firstinspires.ftc.robotcontroller.internal.Cisco;

import android.util.Log;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.Locale;

/**
 * Created by sahith on 12/10/17.
 */

public class ciscoAutoBlue extends LinearOpMode {
    Servo cat;
    ColorSensor jewelSensor;
    DcMotor fr, fl, br, bl;
    DcMotor rgrab, lgrab;
    Servo rflip, lflip, stopper;

    BNO055IMU imu;
    Orientation lastAngles;

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables trackables;
    VuforiaTrackable trackable;
    VuforiaTrackableDefaultListener listener;
    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String V_KEY = "Aelee1z/////AAAAGSOzCHm55k0ymtB98J5uxqsJ3VZ0L7SyL2P9mk" +
            "wAcASPo5jphdOuL+kEucaxbqfA3GfMKZbO9/7zChpv5oYcZ+yc/T6cl7LjsJVETcd2kf+6W1cby1xBzFB" +
            "wuHi0wHKvxx+PmsKhwA0dLReiaVIkev7aJ8CrAhDfPfkeT8HMXNRvoJ3tkiAkfc9ONcNxNt4XzbXTcslg" +
            "8/+xgHw/q7yhnuWy1hgPizTCcIQr+oWRuukFDC228GuWhTiHDGFqHWMd0dublyymuEgSxpSByagvaHJ73" +
            "/mJbfRCTcBJgmvLmAzv86N6vyycC4pReQxHn8djz8QCicZnEbz9CM10HcaRxqftMFNv0iKnlDH85NJrnyMw";

    String imageDetected;

    final static double CAT_STOW = 0.79;
    final static double CAT_EXTEND = 0.31;

    final static double STRAIGHT_TICKS_PER_INCH = 52.63;
    final static double SIDE_TICKS_PER_INCH = 51.51;

    final static double STOPPER_STOP = 0.959444444444444445;
    final static double STOPPER_DEPOSIT = 0.62000000000000001;
    final static double STOPPER_ZERO = 0.16944444444444452;
    final static double RFLIP_DEPOSIT = 0.7794444444444446;
    final static double RFLIP_ZERO = 0.199444444444444444448;
    final static double RFLIP_GRAB = 0.0500000000000000000044;
    final static double LFLIP_DEPOSIT = 0.040000000000000036;
    final static double LFLIP_ZERO = 0.59000000000000000001;
    final static double LFLIP_GRAB = 0.719444444444444444446;

    final static int DESIRED_BLUE = 11;

    double p_turn = .037;//0.008;
    double i_turn = .002; //.0045; //.003;
    double d_turn = .002; //.04 //.0045;
    double f_turn = .0;
    double pT = 0;
    double pE = 0;
    double tE = 0;

    double dist = 0;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        cat = hardwareMap.servo.get("cat");

        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        rgrab = hardwareMap.dcMotor.get("rintake");
        lgrab = hardwareMap.dcMotor.get("lintake");

        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");
        stopper = hardwareMap.servo.get("stopper");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        rgrab.setPower(0);
        lgrab.setPower(0);

        cat.setPosition(CAT_STOW);

        zero();

        setupVuforia(0);

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        //Coordinates are in millimeters and are based off of the center of the robot

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        showPID();

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        imageDetected = doVuforia();
        jewelAuto(jewelSensor, cat);
        driveBackward(-15 + dist, -0.4);
        sleep(2000);
        doImage(imageDetected);
    }

    //VUFORIA FUNCTIONS-----------------------------------------------------------------------------
    public void setupVuforia(int image) {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        Log.i("setup", "workss");
        parameters.vuforiaLicenseKey = V_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);
        Log.i("test", "works");
        trackables = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        trackable = trackables.get(image);
        trackable.setName("Relic Images");
        trackable.setLocation(createMatrix(0, 500, 0, 0, 0, 0));

        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        listener = (VuforiaTrackableDefaultListener) trackable.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        Log.i("setup", "works");
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix
                (AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String doVuforia() {
        String image = "none";

        trackables.activate();

        while (image == "none") {
            OpenGLMatrix currentLocation = listener.getUpdatedRobotLocation();

            if (currentLocation != null) {
                lastKnownLocation = currentLocation;
            }

            RelicRecoveryVuMark detector = RelicRecoveryVuMark.from(trackable);

            if (listener.isVisible()) {
                if (detector == RelicRecoveryVuMark.LEFT) {
                    image = "L";
                } else if (detector == RelicRecoveryVuMark.CENTER) {
                    image = "C";
                } else if (detector == RelicRecoveryVuMark.RIGHT) {
                    image = "R";
                }
                else {
                    image = "none";
                }
            }
            telemetry.addData("image", image);
            telemetry.update();
        }

        return image;
    }

    public void doImage(String image) {
        // check for robot at the top of the triangle
        if (image.equals("R")) {
            turn(-120, 3);
            driveForward(6, 0.3);
            outtake();
            sleep(500);
            driveBackward(-5, -0.3);
            sleep(1000);
            driveForward(7, 0.3);
            sleep(500);
            driveBackward(-5, -0.3);
        } else if (image.equals("L")) {
            turn(-74, 3);
            driveForward(6, 0.3);
            outtake();
            sleep(500);
            driveBackward(-5, -0.3);
            sleep(1000);
            driveForward(7, 0.3);
            sleep(500);
            driveBackward(-5, -0.3);
        } else {
            turn(-98, 3);
            driveForward(4, 0.3);
            outtake();
            sleep(500);
            driveBackward(-4, -0.3);
            sleep(1000);
            driveForward(5, 0.3);
            sleep(500);
            driveBackward(-4, -0.3);
        }
    }
    //----------------------------------------------------------------------------------------------

    //PID FUNCTIONS---------------------------------------------------------------------------------
    void showPID() {
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            lastAngles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.thirdAngle);
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format
                (Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void startDegreeController(){
        pT = runtime.time();
        pE = 0;
        tE = 0;
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

    public void turn(double degree, double margin) {
        startDegreeController();
        double pYaw = lastAngles.firstAngle;
        while (Math.abs(getDifference(lastAngles.firstAngle, degree)) > margin ||
                Math.abs(pYaw - lastAngles.firstAngle) > .05) {
            double change = degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            if (getDifference(lastAngles.firstAngle, degree) > 0) {
                fr.setPower(0.5 * backPower);
                br.setPower(0.5 * backPower);
                fl.setPower(0.5 * forwardPower);
                bl.setPower(0.5 * forwardPower);
            } else {
                fr.setPower(0.5 * forwardPower);
                br.setPower(0.5 * forwardPower);
                fl.setPower(0.5 * backPower);
                bl.setPower(0.5 * backPower);
            }
            telemetry.addData("hey", "yaw: " + lastAngles.firstAngle);
            telemetry.addData("degree", "degree: " + degree);
            pYaw = lastAngles.firstAngle;
            resetAngles();
            telemetry.update();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        telemetry.addData("Reached", degree);
    }

    void resetAngles() {
        lastAngles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
    //----------------------------------------------------------------------------------------------

    //JEWEL FUNCTION--------------------------------------------------------------------------------
    public void jewelAuto(ColorSensor sensor, Servo servo) {
        servo.setPosition(CAT_EXTEND);
        sleep(1000);
        if (sensor.red() > sensor.blue()) {
            dist = -5;
            driveBackward(-2, -0.35);
            sleep(2000);
            servo.setPosition(CAT_STOW);
            sleep(2000);
        }
        else {
            dist = -3;
            driveForward(1, 0.25);
            servo.setPosition(CAT_STOW);
            sleep(500);
            driveBackward(-11, -1);
            sleep(500);
        }
    }
    //----------------------------------------------------------------------------------------------

    //GLYPH FUNCTIONS-------------------------------------------------------------------------------
//    public void grab(double power) {
//        rgrab.setPower(power);
//        lgrab.setPower(power);
//    }

    public void outtake() {
        deposit();
        sleep(500);
    }

    public void grab() {
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
        stopper.setPosition(STOPPER_STOP);
    }
    public void zero() {
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
        stopper.setPosition(STOPPER_STOP);
    }
    public void deposit() {
        rflip.setPosition(RFLIP_DEPOSIT);
        lflip.setPosition(LFLIP_DEPOSIT);
        stopper.setPosition(STOPPER_DEPOSIT);
    }

//    public void doNextGlyphs(String image) {
//        // check for robot at the top of the triangle
//        if (image.equals("L")) {
//            drive(4, "FORWARD", 0.4);
//            outtake();
//            drive(1, "BACKWARD", 0.1);
//            drive(2, "RIGHT", 0.1);
//            drive(1, "FORWARD", 0.1);
//            outtake();
//            drive(1, "BACKWARD", 0.1);
//        } else if (image.equals("R")) {
//            drive(4, "FORWARD", 0.4);
//            outtake();
//            drive(1, "BACKWARD", 0.1);
//            drive(2, "LEFT", 0.1);
//            drive(1, "FORWARD", 0.1);
//            outtake();
//            drive(1, "BACKWARD", 0.1);
//        } else {
//            turn(-60, 3);
//            drive(4, "FORWARD", 0.4);
//            outtake();
//            drive(1, "BACKWARD", 0.1);
//            turn(-90, 3);
//            drive(4, "LEFT", 0.2);
//            drive(1, "FORWARD", 0.1);
//            outtake();
//            drive(1, "BACKWARD", 0.1);
//        }
//    }
    //----------------------------------------------------------------------------------------------

    //DRIVE FUNCTIONS--------------------------------------------------------------------------------
    public void driveForward(double distance, double maxpower) {
        int ticks;
        int old_ticks = fl.getCurrentPosition();
        ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
        fr.setPower(maxpower);
        fl.setPower(maxpower);
        br.setPower(maxpower);
        bl.setPower(maxpower);
        while (fl.getCurrentPosition() < ticks + old_ticks) {
            telemetry.addData("Fl current position: ", fl.getCurrentPosition());
            telemetry.addData("Fl power: ", fl.getPower());
        }
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void driveBackward(double distance, double maxpower) {
        int ticks;
        int old_ticks = fl.getCurrentPosition();
        ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
        fr.setPower(maxpower);
        fl.setPower(maxpower);
        br.setPower(maxpower);
        bl.setPower(maxpower);
        while (fl.getCurrentPosition() > ticks + old_ticks) {
            telemetry.addData("Fl current position: ", fl.getCurrentPosition());
            telemetry.addData("Fl power: ", fl.getPower());
        }
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    //----------------------------------------------------------------------------------------------
}