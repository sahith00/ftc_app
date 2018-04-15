package org.firstinspires.ftc.robotcontroller.internal.Auto;

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by sahith on 2/12/18.
 */

public class autoMethods extends LinearOpMode {
    DcMotor fr, fl, br, bl;
    DcMotor rintake, lintake;
    DcMotor lift;
    Servo lig;
    Servo cat, knock;
    ColorSensor jewelSensor;
    DistanceSensor distanceSensor;
    Servo rflip, lflip, extendstopper, stopper, bottomgrab, topgrab;

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

    final static double STRAIGHT_TICKS_PER_INCH = 40.00;
    final static double SIDE_TICKS_PER_INCH = 51.5;

    final static double CAT_STOW = 0.619444444444444444445;
    final static double CAT_EXTEND = 0.1894444444444444445;
    final static double KNOCK_RIGHT = .2400000000000000005; //0.56
    final static double KNOCK_LEFT = 1;  //0.18
    final static double KNOCK_CENTER = 0.60000000000000001;
    final static double LIG_STOW = .01999999999999994;
    final static double LIG_HALF_STOW = 0.35944444444444445;

    final static double BOTTOMGRAB_GRAB = 0.0;
    final static double BOTTOMGRAB_STOW = .215;
    final static double TOPGRAB_GRAB = 0.0;
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

    public double p_turn = .052;//0.008;
    public double i_turn = .000; //.0045; //.003;
    public double d_turn = .002; //.04 //.0045;
    public double pT = 0;
    public double pE = 0;
    public double tE = 0;

    public double autoT = 0;

    public double knockJewel = 70;
    public boolean pushJewel = false;
    public double motorpTurn = 0;
    public boolean stuckTurn = true;

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");
        rintake = hardwareMap.dcMotor.get("rintake");
        lintake = hardwareMap.dcMotor.get("lintake");
        lift = hardwareMap.dcMotor.get("lift");
        lig = hardwareMap.servo.get("lig");
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");
        jewelSensor = hardwareMap.get(ColorSensor.class, "jewelSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "jewelSensor");
        rflip = hardwareMap.servo.get("rflip");
        lflip = hardwareMap.servo.get("lflip");
        bottomgrab = hardwareMap.servo.get("bottomgrab");
        topgrab = hardwareMap.servo.get("topgrab");
        stopper = hardwareMap.servo.get("stopper");
        extendstopper = hardwareMap.servo.get("extendstopper");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        rintake.setDirection(DcMotorSimple.Direction.REVERSE);
        lintake.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        resetAngles();

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        rintake.setPower(0);
        lintake.setPower(0);

        knock.setPosition(KNOCK_RIGHT);
        sleep(200);
        cat.setPosition(CAT_STOW);
        sleep(200);
        lig.setPosition(LIG_STOW);
        sleep(200);
        stopper.setPosition(STOPPER_STOW);
        sleep(250);
        extendstopper.setPosition(EXTENDSTOPPER_STOW);
        sleep(250);
        lift.setPower(.5);
        lift.setTargetPosition(0);
        lift.setPower(.0);
        zero();

        setUpVuforia();
    }

    //JEWEL FUNCTION--------------------------------------------------------------------------------
    public void jewelAuto() {
        extend();
        sleep(500);
        if (jewelSensor.blue() > jewelSensor.red()) {
            telemetry.addData("Blue", jewelSensor.blue());
            telemetry.addData("Red", jewelSensor.red());
            telemetry.update();
            left();
            pushJewel = true;
            center();
        } else if (jewelSensor.red() > jewelSensor.blue()) {
            telemetry.addData("Blue", jewelSensor.blue());
            telemetry.addData("Red", jewelSensor.red());
            telemetry.update();
            right();
            pushJewel = false;
            center();
        }
    }

    public void stow() {
        knock.setPosition(KNOCK_CENTER);
        sleep(200);
        cat.setPosition(CAT_STOW);
        sleep(200);
    }
    public void colorDistExtend(double offset) {
        cat.setPosition(CAT_EXTEND + offset);
        sleep(200);
    }
    public void colorExtendClose() {
        cat.setPosition(CAT_EXTEND + 0.15);
        sleep(200);
    }
    public void distExtendClose() {
        cat.setPosition(CAT_EXTEND + 0.15);
        sleep(200);
    }
    public void colorDistExtendFar() {
        cat.setPosition(CAT_EXTEND + 0.29);
        sleep(200);
    }
    public void extend() {
        knock.setPosition(KNOCK_CENTER);
        sleep(200);
        cat.setPosition(CAT_EXTEND);
        sleep(200);
    }

    public void right() {
        knock.setPosition(KNOCK_RIGHT);
        sleep(200);
    }

    public void left() {
        knock.setPosition(KNOCK_LEFT);
        sleep(200);
        // driveDistance(2.5, .5);
    }

    public void center() {
        knock.setPosition(KNOCK_CENTER);
        sleep(200);
    }

    //----------------------------------------------------------------------------------------------

    //GLYPH FUNCTIONS-------------------------------------------------------------------------------
    public void doImage(String team, String image, double rturn, double lturn, double cturn, boolean close) {
        double rdistance = 0, ldistance = 0, cdistance = 0;
        if (close) {
            if(team.equals("RED")) {
                rdistance = 13;
                ldistance = 17;
                cdistance = 15;
            }
            else if(team.equals("BLUE")) {
                rdistance = 17;
                ldistance = 13;
                cdistance = 15;
            }
        } else {
            if(team.equals("RED")) {
                rdistance = 5;
                ldistance = 10.5;
                cdistance = 7;
            }
            else if(team.equals("BLUE")) {
                rdistance = 10.5+1;
                ldistance = 5.75;
                cdistance = 7;
            }
        }
        double tempTime = runtime.milliseconds();
        if(team.equals("BLUE")) {
            while ((jewelSensor.blue() < 100 || tempTime + 4000 < runtime.milliseconds()) && opModeIsActive()) {
                drive(-0.2);
                telemetry.addData("Motor power", fl.getPower());
                telemetry.addData("distance", String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
                telemetry.update();
            }
            drive(0.0);
            if (close) {
                distExtendClose();
            }
            while (checkDistance() != 0 && opModeIsActive()) {
                strafe(0.5);
            }
            strafe(0.0);
        }
        else if(team.equals("RED")) {
            while ((jewelSensor.red() < 100 || tempTime + 4000 < runtime.milliseconds()) && opModeIsActive()) {
                drive(0.2);
                telemetry.addData("Motor power", fl.getPower());
                telemetry.addData("distance", String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
                telemetry.update();
            }
            drive(0.0);
            if (close) {
                distExtendClose();
            }
            while (checkDistance() != 0 && opModeIsActive()) {
                strafe(0.5);
            }
            strafe(0.0);
        }

        stow();
        stopper.setPosition(STOPPER_STOW);
        sleep(250);
        extendstopper.setPosition(EXTENDSTOPPER_STOW);
        sleep(250);
       /* if (pushJewel) {
            turn(knockJewel, 15);
          //  driveDistance(5, 0.5);
          //  driveDistance(-5, -0.5);
            pushJewel = false;
        }*/

        if (image.equals("R")) {
            if(team.equals("BLUE")&&close) {
                driveDistance(-2, -0.4, false);
                turn(rturn, 1);
            }
            else {
                turn(rturn, 1);
            }
            outtake();
            driveDistance(rdistance, 0.5, true);
            //sleep(500);
            dropGlyph();
        } else if (image.equals("L")) {
            turn(lturn, 1);
            outtake();
            driveDistance(ldistance, 0.5, true);
            //sleep(500);
            dropGlyph();
        } else {
            turn(cturn, 1);
            outtake();
            driveDistance(cdistance, 0.5, true);
            //sleep(500);
            dropGlyph();
        }
    }

    public double checkDistance() {
        double dist = distanceSensor.getDistance(DistanceUnit.CM);
        if(dist != dist) {
            return 0;
        }
        return dist;
    }

    public String doVuforia() {
        String image = "none";

        trackables.activate();

        while ((image == "none" && autoT + 15000 > runtime.milliseconds()) && opModeIsActive()) {
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
        }

        if(image.equals("none")) {
            image = "L";
        }

        trackables.deactivate();

        return image;
    }

    public void glyphAutoClose(String image, double rturn, double lturn, double cturn, double cryptoturn) {
        double init_ticks;
        driveDistance(-5, -0.4, false);
        grab();
        turn(cryptoturn, 2.5); //with flipper towards cryptobox
        init_ticks = bl.getCurrentPosition();
        grabGlyph(1.0);
        drive(-0.4);
        sleep(3000);
        drive(0);
        turnForGlyph();
        grabGlyph(-1.0);
        sleep(1000);
        grabGlyph(0.0);
        turn(cryptoturn, 2.5);
        driveDistance((init_ticks - bl.getCurrentPosition()) / STRAIGHT_TICKS_PER_INCH, 0.5, false);
        extend();
        if(image.equals("R") || image.equals("C")) {
            if(checkDistance() == 0) {
                while(checkDistance() == 0) {
                    strafe(0.4);
                }
                strafe(0.0);
                sleep(500);
                stow();
                strafe(-0.2);
                sleep(1000);
                strafe(0.0);
            }
            else {
                stow();
                strafe(-0.2);
                sleep(1000);
                strafe(0.0);
            }
        }
        else {
            if(checkDistance() == 0) {
                while(checkDistance() == 0) {
                    strafe(-0.4);
                }
                strafe(0.0);
                sleep(500);
                stow();
                strafe(0.2);
                sleep(1000);
                strafe(0.0);
            }
            else {
                stow();
                strafe(0.2);
                sleep(1000);
                strafe(0.0);
            }
        }
        driveDistance(-5, -0.4, false);
        outtake();
        sleep(500);
        driveDistance(5, 0.4, true);
        dropGlyph();
    }

    public void glyphAutoFar(String image, String team, double glyphturn, double lturn, double rturn) {
        extendstopper.setPosition(EXTENDSTOPPER_STOP);
        turn(-90, 3);
        if (image.equals("R")) {
            strafeDist(3, 0.5);
            driveDist(-9.5, -0.4, false);
            colorDistExtend(.17);
        } else {
            driveDist(-11.5, -0.4, false);
            colorDistExtend(.17);
        }
        while (jewelSensor.blue() < 100 && opModeIsActive()) {
            drive(-0.2);
            telemetry.addData("Motor power", fl.getPower());
            telemetry.addData("distance", String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
        drive(0.0);
        double rdistance = 0, ldistance = 0;
        if(team.equals("RED")) {
            rdistance = 5;
            ldistance = 10.5;
        }
        else if(team.equals("BLUE")) {
            rdistance = 10.5;
            ldistance = 5.75;
        }
        stow();
        turn(glyphturn, 4);
        grab();
        grabGlyph(1.0);
        double init_ticks = bl.getCurrentPosition();
        drive(-0.6);
        sleep(1500);
        drive(0.0);
        sleep(750);
        turnForGlyph();
        turn(glyphturn, 1);
        double new_ticks = bl.getCurrentPosition();
        lift.setPower(.7);
        lift.setTargetPosition(500);
        driveDistance(((new_ticks-init_ticks)/STRAIGHT_TICKS_PER_INCH) + ldistance, 0.4, false);
        sleep(500);
        dropGlyph();
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public void setUpVuforia() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = V_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);
        trackables = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        trackable = trackables.get(0);
        trackable.setName("Relic Images");
        trackable.setLocation(createMatrix(0, 500, 0, 0, 0, 0));

        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        listener = (VuforiaTrackableDefaultListener) trackable.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }

    public void dropGlyph() {
        topgrab.setPosition(TOPGRAB_STOW);
        bottomgrab.setPosition(BOTTOMGRAB_STOW);
        //  sleep(500);
        driveDistance(-5, -0.4, true);
        /*sleep(500);
        driveDistance(6.5, 0.4, true);
        sleep(500);
        driveDistance(-5, -0.4, true);*/
    }

    public void outtake() {
        deposit();
        sleep(750);
    }

    public void grab() {
        extendstopper.setPosition(EXTENDSTOPPER_STOP);
        stopper.setPosition(STOPPER_STOP);
        rflip.setPosition(RFLIP_GRAB);
        lflip.setPosition(LFLIP_GRAB);
        bottomgrab.setPosition(BOTTOMGRAB_STOW);
        topgrab.setPosition(TOPGRAB_STOW);
    }
    public void zero() {
        rflip.setPosition(RFLIP_ZERO);
        lflip.setPosition(LFLIP_ZERO);
        bottomgrab.setPosition(BOTTOMGRAB_GRAB);
        topgrab.setPosition(TOPGRAB_GRAB);
        extendstopper.setPosition(EXTENDSTOPPER_STOP);
        stopper.setPosition(STOPPER_STOW);
    }
    public void deposit() {
        bottomgrab.setPosition(BOTTOMGRAB_GRAB);
        topgrab.setPosition(TOPGRAB_GRAB);
        stopper.setPosition(STOPPER_STOW);
        extendstopper.setPosition(EXTENDSTOPPER_STOW);
        rflip.setPosition(RFLIP_DEPOSIT);
        lflip.setPosition(LFLIP_DEPOSIT);
    }

    public void grabGlyph(double power) {
        rintake.setPower(power);
        lintake.setPower(power);
    }

    public void turnForGlyph() {
        double degree = lastAngles.firstAngle;
        turn(degree - 15, 5);
        turn(degree, 2.5);
        turn(degree + 15, 5);
        turn(degree, 2.5);
    }
    //----------------------------------------------------------------------------------------------

    //DRIVE FUNCTIONS-------------------------------------------------------------------------------
    public void driveDist(double distance, double maxpower, boolean timer) {
        double tempTime = runtime.milliseconds();
        if (distance > 0) {
            int ticks;
            int old_ticks = bl.getCurrentPosition();
            ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
            fr.setPower(maxpower);
            fl.setPower(maxpower);
            br.setPower(maxpower);
            bl.setPower(maxpower);
            while ((bl.getCurrentPosition() < (ticks + old_ticks)) && opModeIsActive()) {
                if(timer) {
                    if(tempTime + 5000 < runtime.milliseconds()) {
                        break;
                    }
                }
                if (autoT + 28000 < runtime.milliseconds()) {
                    end();
                }
            }
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
        else if (distance < 0) {
            int ticks;
            int old_ticks = bl.getCurrentPosition();
            ticks = (int) (STRAIGHT_TICKS_PER_INCH * distance);
            fr.setPower(maxpower);
            fl.setPower(maxpower);
            br.setPower(maxpower);
            bl.setPower(maxpower);
            while ((bl.getCurrentPosition() > (ticks + old_ticks)) && opModeIsActive()) {
                if(timer) {
                    if(tempTime + 5000 < runtime.milliseconds()) {
                        break;
                    }
                }
                if (autoT + 28000 < runtime.milliseconds()) {
                    end();
                }
            }
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
    }

    public void driveDistance(double distance, double maxpower, boolean timer) {
        driveDist(0.5 * distance, maxpower/1, timer);
        driveDist(0.3 * distance, maxpower/2, timer);
        driveDist(0.2 * distance, maxpower/3, timer);
    }

    public void drive(double maxpower) {
        fr.setPower(maxpower);
        fl.setPower(maxpower);
        br.setPower(maxpower);
        bl.setPower(maxpower);
    }

    public void strafe(double maxpower) {
        fr.setPower(maxpower);
        fl.setPower(-maxpower);
        br.setPower(-maxpower);
        bl.setPower(maxpower);
    }
    public void strafeDist(double distance, double maxpower) {
        if (distance > 0) {
            int ticks;
            int old_ticks = bl.getCurrentPosition();
            ticks = (int) (SIDE_TICKS_PER_INCH * distance);
            strafe(maxpower);
            while ((bl.getCurrentPosition() < (ticks + old_ticks)) && opModeIsActive()) {
               /* if (autoT + 28000 < runtime.milliseconds()) {
                    end();
                }*/
            }
            strafe(0.0);
        }
        else if (distance < 0) {
            int ticks;
            int old_ticks = bl.getCurrentPosition();
            ticks = (int) (SIDE_TICKS_PER_INCH * distance);
            strafe(maxpower);
            while ((bl.getCurrentPosition() > (ticks + old_ticks)) && opModeIsActive()) {
              /*  if (autoT + 28000 < runtime.milliseconds()) {
                    end();
                }*/
            }
            strafe(0.0);
        }
    }

    public void turn(double degree, double margin) {
        startDegreeController();
        stuckTurn = true;
        double pYaw = lastAngles.firstAngle;
        while ((Math.abs(getDifference(lastAngles.firstAngle, degree)) > margin ||
                Math.abs(pYaw - lastAngles.firstAngle) > .05) && opModeIsActive()) {
            double change = degreeController(degree);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            if (getDifference(lastAngles.firstAngle, degree) > 0) {
                fr.setPower(0.65 * backPower);
                br.setPower(0.65 * backPower);
                fl.setPower(0.65 * forwardPower);
                bl.setPower(0.65 * forwardPower);
            } else {
                fr.setPower(0.65 * forwardPower);
                br.setPower(0.65 * forwardPower);
                fl.setPower(0.65 * backPower);
                bl.setPower(0.65 * backPower);
            }
            pYaw = lastAngles.firstAngle;
            resetAngles();
            if(autoT + 28000 < runtime.milliseconds()) {
                end();
            }
            telemetry.addData("Current Yaw: ", pYaw);
            telemetry.addData("Desired Yaw: ", degree);
            telemetry.update();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
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
        telemetry.addData("PID turn", "e: " + e);
        telemetry.addData("PID Turn", "i: " + i_turn * tE);
        telemetry.addData("PID Turn", "d: " + d_turn * dE / dT);
        telemetry.addData("PID Turn", "p: " + p_turn * e);
        if ((dE/dT) == 0.0) {
            if (stuckTurn) {
                motorpTurn = ans;
                stuckTurn = false;
            }
            if (motorpTurn > 0) {
                motorpTurn += .015;
            } else {
                motorpTurn -= .015;
            }
            ans = motorpTurn;
        }
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

    public void end() {
        outtake();
        fr.setPower(-0.2);
        br.setPower(-0.2);
        fl.setPower(-0.2);
        bl.setPower(-0.2);
        sleep(500);
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        while(opModeIsActive()){}
    }
}