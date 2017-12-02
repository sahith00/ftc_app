package org.firstinspires.ftc.robotcontroller.internal.Google;

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Tests.NavX;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

/**
 * Created by sahith on 11/2/17.
 */
public class googleAutoBlue extends LinearOpMode {
    Servo cat, knock;
    CRServo lgrab, rgrab, lbrush, rbrush;
    ColorSensor jewelSensor;
    DcMotor fr, fl, br, bl;
    NavX navX;

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables trackables;
    VuforiaTrackable trackable;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String V_KEY = "Aelee1z/////AAAAGSOzCHm55k0ymtB98J5uxqsJ3VZ0L7SyL2P9mkwAcASPo5jphdOuL+kEucaxbqfA3GfMKZbO9/7zChpv5oYcZ+yc/T6cl7LjsJVETcd2kf+6W1cby1xBzFBwuHi0wHKvxx+PmsKhwA0dLReiaVIkev7aJ8CrAhDfPfkeT8HMXNRvoJ3tkiAkfc9ONcNxNt4XzbXTcslg8/+xgHw/q7yhnuWy1hgPizTCcIQr+oWRuukFDC228GuWhTiHDGFqHWMd0dublyymuEgSxpSByagvaHJ73/mJbfRCTcBJgmvLmAzv86N6vyycC4pReQxHn8djz8QCicZnEbz9CM10HcaRxqftMFNv0iKnlDH85NJrnyMw";

    String imageDetected;

    final static double CAT_STOW = 0.66;
    final static double CAT_EXTEND = 0.35;

    final static double KNOCK_CENTER = 0.38;
    final static double KNOCK_LEFT = 0.32;
    final static double KNOCK_RIGHT = 0.44;

    final static double STRAIGHT_TICKS_PER_INCH = 49.23;
    final static double SIDE_TICKS_PER_INCH = 51.51;

    double p_turn = .025;//0.008;
    double i_turn = .00; //.0045; //.003;
    double d_turn = .0045; //.04 //.0045;
    double f_turn = .0;
    double pT = 0;
    double pE = 0;
    double tE = 0;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        navX = new NavX(hardwareMap);

        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");

        rgrab = hardwareMap.crservo.get("rgrab");
        lgrab = hardwareMap.crservo.get("lgrab");
        rbrush = hardwareMap.crservo.get("rbrush");
        lbrush = hardwareMap.crservo.get("lbrush");

        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        cat.setPosition(CAT_STOW);    //stow
        knock.setPosition(KNOCK_CENTER);    //center

        setupVuforia(0);

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);             // Coordinates are in millimeters and are based off of the center of the robot

        //navX.start();

        waitForStart();

        cat.setPosition(CAT_EXTEND);      //extend
        sleep(1000);
        jewelAuto(jewelSensor, knock);
        cat.setPosition(CAT_STOW);     //stow
        sleep(1000);

        imageDetected = doVuforia();
        if(imageDetected == "L") {
            doImageL();
        }
        else if(imageDetected == "R") {
            doImageR();
        }
        else if(imageDetected == "C") {
            doImageC();
        }

        telemetry.update();
    }

    //---------------------------------------------------------------------------------------------
    // Jewel function
    public void jewelAuto(ColorSensor sensor, Servo servo) {
        if (sensor.red() > sensor.blue()) {
            servo.setPosition(KNOCK_LEFT);
            sleep(1000);
        } else if (sensor.blue() > sensor.red()) {
            servo.setPosition(KNOCK_RIGHT);
            sleep(1000);
        }
        servo.setPosition(KNOCK_CENTER);
        sleep(1000);
        telemetry.addData("Red value", sensor.red());
        telemetry.addData("Blue value", sensor.blue());
        telemetry.update();
    }
    //---------------------------------------------------------------------------------------------



    //---------------------------------------------------------------------------------------------
    // Vuforia functions
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
        }

        return image;
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public void doImageL() {
        drive(26, "FORWARD", 0.7);
        turn(-90, 2);
        drive(10, "FORWARD", 0.4);
        outtake();
        sleep(2500);
        drive(3, "BACKWARD", 0.4);
    }

    public void doImageC() {
        drive(30, "FORWARD", 0.7);
        turn(-90, 2);
        drive(10, "FORWARD", 0.55);
        turn(-90, 2);
        outtake();
        sleep(2500);
        drive(3, "BACKWARD", 0.4);
    }

    public void doImageR() {
        drive(34, "FORWARD", 0.7);
        turn(-90, 2);
        drive(10, "FORWARD", 0.4);
        outtake();
        sleep(2500);
        drive(3, "BACKWARD", 0.4);
    }
    //---------------------------------------------------------------------------------------------


    //---------------------------------------------------------------------------------------------
    // Drive Functions
    public void drive(double distance, String direction, double maxpower) {
        int ticks;
        int old_ticks = fr.getCurrentPosition();
        double multi, old_multi;
        if (direction == "FORWARD") {
            ticks = (int)(STRAIGHT_TICKS_PER_INCH * distance);
            multi = 0.15;
            old_multi = 0.15;
            fr.setPower(0.3*maxpower);
            fl.setPower(0.3*maxpower);
            br.setPower(0.3*maxpower);
            bl.setPower(0.3*maxpower);
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.5*maxpower);
            fl.setPower(0.5*maxpower);
            br.setPower(0.5*maxpower);
            bl.setPower(0.5*maxpower);
            multi = 0.3;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.75*maxpower);
            fl.setPower(0.75*maxpower);
            br.setPower(0.75*maxpower);
            bl.setPower(0.75*maxpower);
            old_multi = multi;
            multi = 0.45;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(1*maxpower);
            fl.setPower(1*maxpower);
            br.setPower(1*maxpower);
            bl.setPower(1*maxpower);
            old_multi = multi;
            multi = 0.6;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.75*maxpower);
            fl.setPower(0.75*maxpower);
            br.setPower(0.75*maxpower);
            bl.setPower(0.75*maxpower);
            old_multi = multi;
            multi = 0.75;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.5*maxpower);
            fl.setPower(0.5*maxpower);
            br.setPower(0.5*maxpower);
            bl.setPower(0.5*maxpower);
            old_multi = multi;
            multi = 0.85;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.3*maxpower);
            fl.setPower(0.3*maxpower);
            br.setPower(0.3*maxpower);
            bl.setPower(0.3*maxpower);
            old_multi = multi;
            multi = 1;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
            telemetry.addData("flmotore", fl.getCurrentPosition());
            telemetry.addData("frmotore", fr.getCurrentPosition());
            telemetry.addData("blmotore", bl.getCurrentPosition());
            telemetry.addData("brmotore", br.getCurrentPosition());
            telemetry.addData("flmotorp", fl.getPower());
            telemetry.addData("frmotorp", fr.getPower());
            telemetry.addData("blmotorp", bl.getPower());
            telemetry.addData("brmotorp", br.getPower());
            telemetry.update();
        }
        else if (direction == "RIGHT") {
            ticks = (int)(STRAIGHT_TICKS_PER_INCH * distance);
            multi = 0.15;
            old_multi = 0.15;
            fr.setPower(-0.3*maxpower);
            fl.setPower(0.3*maxpower);
            br.setPower(0.3*maxpower);
            bl.setPower(-0.3*maxpower);
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.5*maxpower);
            fl.setPower(0.5*maxpower);
            br.setPower(0.5*maxpower);
            bl.setPower(-0.5*maxpower);
            multi = 0.3;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.75*maxpower);
            fl.setPower(0.75*maxpower);
            br.setPower(0.75*maxpower);
            bl.setPower(-0.75*maxpower);
            old_multi = multi;
            multi = 0.45;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-1*maxpower);
            fl.setPower(1*maxpower);
            br.setPower(1*maxpower);
            bl.setPower(-1*maxpower);
            old_multi = multi;
            multi = 0.6;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.75*maxpower);
            fl.setPower(0.75*maxpower);
            br.setPower(0.75*maxpower);
            bl.setPower(-0.75*maxpower);
            old_multi = multi;
            multi = 0.75;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.5*maxpower);
            fl.setPower(0.5*maxpower);
            br.setPower(0.5*maxpower);
            bl.setPower(-0.5*maxpower);
            old_multi = multi;
            multi = 0.85;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.3*maxpower);
            fl.setPower(0.3*maxpower);
            br.setPower(0.3*maxpower);
            bl.setPower(-0.3*maxpower);
            old_multi = multi;
            multi = 1;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
            telemetry.addData("flmotore", fl.getCurrentPosition());
            telemetry.addData("frmotore", fr.getCurrentPosition());
            telemetry.addData("blmotore", bl.getCurrentPosition());
            telemetry.addData("brmotore", br.getCurrentPosition());
            telemetry.addData("flmotorp", fl.getPower());
            telemetry.addData("frmotorp", fr.getPower());
            telemetry.addData("blmotorp", bl.getPower());
            telemetry.addData("brmotorp", br.getPower());
            telemetry.update();
        }
        else if (direction == "BACKWARD") {
            ticks = (int)(SIDE_TICKS_PER_INCH * distance);
            multi = 0.15;
            old_multi = 0.15;
            fr.setPower(-0.3*maxpower);
            fl.setPower(-0.3*maxpower);
            br.setPower(-0.3*maxpower);
            bl.setPower(-0.3*maxpower);
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.5*maxpower);
            fl.setPower(-0.5*maxpower);
            br.setPower(-0.5*maxpower);
            bl.setPower(-0.5*maxpower);
            multi = 0.3;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.75*maxpower);
            fl.setPower(-0.75*maxpower);
            br.setPower(-0.75*maxpower);
            bl.setPower(-0.75*maxpower);
            old_multi = multi;
            multi = 0.45;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-1*maxpower);
            fl.setPower(-1*maxpower);
            br.setPower(-1*maxpower);
            bl.setPower(-1*maxpower);
            old_multi = multi;
            multi = 0.6;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.75*maxpower);
            fl.setPower(-0.75*maxpower);
            br.setPower(-0.75*maxpower);
            bl.setPower(-0.75*maxpower);
            old_multi = multi;
            multi = 0.75;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.5*maxpower);
            fl.setPower(-0.5*maxpower);
            br.setPower(-0.5*maxpower);
            bl.setPower(-0.5*maxpower);
            old_multi = multi;
            multi = 0.85;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(-0.3*maxpower);
            fl.setPower(-0.3*maxpower);
            br.setPower(-0.3*maxpower);
            bl.setPower(-0.3*maxpower);
            old_multi = multi;
            multi = 1;
            while(fr.getCurrentPosition() > (-(ticks+old_ticks) * multi) && fr.getCurrentPosition() <= (-(ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
            telemetry.addData("flmotore", fl.getCurrentPosition());
            telemetry.addData("frmotore", fr.getCurrentPosition());
            telemetry.addData("blmotore", bl.getCurrentPosition());
            telemetry.addData("brmotore", br.getCurrentPosition());
            telemetry.addData("flmotorp", fl.getPower());
            telemetry.addData("frmotorp", fr.getPower());
            telemetry.addData("blmotorp", bl.getPower());
            telemetry.addData("brmotorp", br.getPower());
            telemetry.update();
        }
        else if (direction == "LEFT") {
            ticks = (int)(SIDE_TICKS_PER_INCH * distance);
            multi = 0.15;
            old_multi = 0.15;
            fr.setPower(0.3*maxpower);
            fl.setPower(-0.3*maxpower);
            br.setPower(-0.3*maxpower);
            bl.setPower(0.3*maxpower);
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.5*maxpower);
            fl.setPower(-0.5*maxpower);
            br.setPower(-0.5*maxpower);
            bl.setPower(0.5*maxpower);
            multi = 0.3;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.75*maxpower);
            fl.setPower(-0.75*maxpower);
            br.setPower(-0.75*maxpower);
            bl.setPower(0.75*maxpower);
            old_multi = multi;
            multi = 0.45;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(1*maxpower);
            fl.setPower(-1*maxpower);
            br.setPower(-1*maxpower);
            bl.setPower(1*maxpower);
            old_multi = multi;
            multi = 0.6;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.75*maxpower);
            fl.setPower(-0.75*maxpower);
            br.setPower(-0.75*maxpower);
            bl.setPower(0.75*maxpower);
            old_multi = multi;
            multi = 0.75;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.5*maxpower);
            fl.setPower(-0.5*maxpower);
            br.setPower(-0.5*maxpower);
            bl.setPower(0.5*maxpower);
            old_multi = multi;
            multi = 0.85;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0.3*maxpower);
            fl.setPower(-0.3*maxpower);
            br.setPower(-0.3*maxpower);
            bl.setPower(0.3*maxpower);
            old_multi = multi;
            multi = 1;
            while(fr.getCurrentPosition() < ((ticks+old_ticks) * multi) && fr.getCurrentPosition() >= ((ticks+old_ticks) * old_multi)) {
                telemetry.addData("flmotore", fl.getCurrentPosition());
                telemetry.addData("frmotore", fr.getCurrentPosition());
                telemetry.addData("blmotore", bl.getCurrentPosition());
                telemetry.addData("brmotore", br.getCurrentPosition());
                telemetry.addData("flmotorp", fl.getPower());
                telemetry.addData("frmotorp", fr.getPower());
                telemetry.addData("blmotorp", bl.getPower());
                telemetry.addData("brmotorp", br.getPower());
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
            telemetry.addData("flmotore", fl.getCurrentPosition());
            telemetry.addData("frmotore", fr.getCurrentPosition());
            telemetry.addData("blmotore", bl.getCurrentPosition());
            telemetry.addData("brmotore", br.getCurrentPosition());
            telemetry.addData("flmotorp", fl.getPower());
            telemetry.addData("frmotorp", fr.getPower());
            telemetry.addData("blmotorp", bl.getPower());
            telemetry.addData("brmotorp", br.getPower());
            telemetry.update();
        }
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

    public void startDegreeController(){
        pT = runtime.time();
        pE = 0;
        tE = 0;
    }

    public double degreeController(double degree, NavX navX){
        double ans = 0;
        double e = Math.abs(getDifference(navX.getYaw(), degree));
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

    public void turn(double degree, double margin) {
        startDegreeController();
        double pYaw = navX.yaw;
        while (Math.abs(getDifference(navX.yaw, degree)) > margin || Math.abs(pYaw - navX.yaw) > .05) {
            Log.i("debug: ", "yaw: " + navX.yaw);
            Log.i("debug: ", "difference: " + getDifference(navX.yaw, degree));
            Log.i("debug", "target: " + degree + " cur: " + navX.yaw);
            double change = degreeController(degree, navX);
            double forwardPower = Range.clip(change, -1, 1);
            double backPower = Range.clip(-change, -1, 1);
            Log.i("powers", "forward: " + forwardPower);
            Log.i("powers", "backward: " + backPower);
            Log.i("debug: ", "difference: " + getDifference(navX.yaw, degree));
            if (getDifference(navX.yaw, degree) > 0) {
                fr.setPower(backPower);
                br.setPower(backPower);
                fl.setPower(forwardPower);
                bl.setPower(forwardPower);
            } else {
                fr.setPower(forwardPower);
                br.setPower(forwardPower);
                fl.setPower(backPower);
                bl.setPower(backPower);
            }
            Log.i("hey", "yaw: " + navX.yaw);
            Log.i("degree", "degree: " + degree);
            pYaw = navX.yaw;
            navX.update();
        }
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
    }
    //---------------------------------------------------------------------------------------------


    //---------------------------------------------------------------------------------------------
    // Grabber functions
    public void outtake() {
        rgrab.setPower(-1);
        rbrush.setPower(-1);
        lgrab.setPower(1);
        lbrush.setPower(1);
        sleep(4000);
        rgrab.setPower(0);
        rbrush.setPower(0);
        lgrab.setPower(0);
        lbrush.setPower(0);
    }

    public void intake() {
        rgrab.setPower(1);
        rbrush.setPower(1);
        lgrab.setPower(-1);
        lbrush.setPower(-1);
        sleep(4000);
        rgrab.setPower(0);
        rbrush.setPower(0);
        lgrab.setPower(0);
        lbrush.setPower(0);
    }
}
