package org.firstinspires.ftc.robotcontroller.internal.Google;

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
public class googleAutoRed extends LinearOpMode {
    Servo cat, knock;
    ColorSensor jewelSensor;
    DcMotor fr, fl, br, bl;

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables trackables;
    VuforiaTrackable trackable;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String V_KEY = "Aelee1z/////AAAAGSOzCHm55k0ymtB98J5uxqsJ3VZ0L7SyL2P9mkwAcASPo5jphdOuL+kEucaxbqfA3GfMKZbO9/7zChpv5oYcZ+yc/T6cl7LjsJVETcd2kf+6W1cby1xBzFBwuHi0wHKvxx+PmsKhwA0dLReiaVIkev7aJ8CrAhDfPfkeT8HMXNRvoJ3tkiAkfc9ONcNxNt4XzbXTcslg8/+xgHw/q7yhnuWy1hgPizTCcIQr+oWRuukFDC228GuWhTiHDGFqHWMd0dublyymuEgSxpSByagvaHJ73/mJbfRCTcBJgmvLmAzv86N6vyycC4pReQxHn8djz8QCicZnEbz9CM10HcaRxqftMFNv0iKnlDH85NJrnyMw";

    String imageDetected;

    double CAT_STOW = 0.34;
    double CAT_EXTEND = 0.61;

    double KNOCK_CENTER = 0.38;
    double KNOCK_LEFT = 0.33;
    double KNOCK_RIGHT = 0.43;

    @Override
    public void runOpMode() throws InterruptedException {
        cat = hardwareMap.servo.get("cat");
        knock = hardwareMap.servo.get("knock");

        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        fr = hardwareMap.dcMotor.get("frdrive");
        fl = hardwareMap.dcMotor.get("fldrive");
        br = hardwareMap.dcMotor.get("brdrive");
        bl = hardwareMap.dcMotor.get("bldrive");

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cat.setPosition(CAT_STOW);    //stow
        knock.setPosition(KNOCK_CENTER);    //center

        setupVuforia(0);

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);             // Coordinates are in millimeters and are based off of the center of the robot

        waitForStart();

        cat.setPosition(CAT_EXTEND);      //extend
        sleep(1000);
        jewelAuto(jewelSensor, knock);
        cat.setPosition(CAT_STOW);     //stow
        sleep(1000);

        drive();
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

    public void jewelAuto(ColorSensor sensor, Servo servo) {
        if (sensor.red() > sensor.blue()) {
            servo.setPosition(KNOCK_LEFT);    //left
            sleep(1000);
        } else if (sensor.blue() > sensor.red()) {
            servo.setPosition(KNOCK_RIGHT);      //right
            sleep(1000);
        }
        servo.setPosition(KNOCK_CENTER);
        sleep(1000);
        telemetry.addData("Red value", sensor.red());
        telemetry.addData("Blue value", sensor.blue());
    }

    public void drive(/*for a certain distance*/) {
        fr.setPower(1.0);
        fl.setPower(1.0);
        br.setPower(1.0);
        bl.setPower(1.0);
    }

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
        String image = "";

        trackables.activate();

        OpenGLMatrix currentLocation = listener.getUpdatedRobotLocation();

        if(currentLocation != null) {
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
        }

        return image;
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

    public void doImageL() {
        Log.i("Hi", "Heee");
    }

    public void doImageC() {
        Log.i("Hi", "Hee");
    }

    public void doImageR() {
        Log.i("Hi", "He");
    }
}
