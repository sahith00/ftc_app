package org.firstinspires.ftc.robotcontroller.internal.RI3D;

import android.content.pm.ActivityInfo;
import android.graphics.Camera;
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
 * Created by sahith on 9/10/17.
 */
public class RI3DAuto extends LinearOpMode {
    DcMotor frdrive, fldrive, brdrive, bldrive;

    ColorSensor jewelSensor;
    Servo jewelServo;

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables trackables;
    VuforiaTrackable trackable;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String V_KEY = "Aelee1z/////AAAAGSOzCHm55k0ymtB98J5uxqsJ3VZ0L7SyL2P9mkwAcASPo5jphdOuL+kEucaxbqfA3GfMKZbO9/7zChpv5oYcZ+yc/T6cl7LjsJVETcd2kf+6W1cby1xBzFBwuHi0wHKvxx+PmsKhwA0dLReiaVIkev7aJ8CrAhDfPfkeT8HMXNRvoJ3tkiAkfc9ONcNxNt4XzbXTcslg8/+xgHw/q7yhnuWy1hgPizTCcIQr+oWRuukFDC228GuWhTiHDGFqHWMd0dublyymuEgSxpSByagvaHJ73/mJbfRCTcBJgmvLmAzv86N6vyycC4pReQxHn8djz8QCicZnEbz9CM10HcaRxqftMFNv0iKnlDH85NJrnyMw";

    @Override
    public void runOpMode() throws InterruptedException {
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");
        jewelServo = hardwareMap.servo.get("jewelServo");

        frdrive = hardwareMap.dcMotor.get("frdrive");
        fldrive = hardwareMap.dcMotor.get("fldrive");
        brdrive = hardwareMap.dcMotor.get("brdrive");
        bldrive = hardwareMap.dcMotor.get("bldrive");

        boolean imageLDetected = false, imageCDetected = false, imageRDetected = false;
        setupVuforia(0);
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);             // Coordinates are in millimeters and are based off of the center of the robot

        waitForStart();

        jewelAuto(jewelSensor, jewelServo);
        vuforiaAuto(imageLDetected, imageCDetected, imageRDetected);
        telemetry.update();
    }

    public void mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, double multiplier) {
        double joyly = -(gamepad1.left_stick_y);
        double joylx = gamepad1.left_stick_x;
        double joyrx = gamepad1.right_stick_x;
        double vd = Math.hypot(joyly, joylx);
        double theta = Math.atan2(joylx, joyly);
        double v0 = joyrx;
        double v1 = vd*Math.sin(theta+(Math.PI/4))+v0; //fl
        double v2 = vd*Math.cos(theta + (Math.PI / 4))-v0; //fr
        double v3 = vd*Math.cos(theta+(Math.PI/4))+v0; //bl
        double v4 = vd*Math.sin(theta + (Math.PI / 4))-v0; //br
        double temp_max = Math.max(Math.abs(v1), Math.abs(v2));
        double temp_max2 = Math.max(temp_max, Math.abs(v3));
        double max = Math.max(temp_max2, Math.abs(v4));

        if (max != 0) {
            fl.setPower(multiplier * (v1/max));
            fr.setPower(multiplier * (v2/max));
            bl.setPower(multiplier * (v3/max));
            br.setPower(multiplier * (v4/max));
        } else {
            fl.setPower(0.0);
            fr.setPower(0.0);
            bl.setPower(0.0);
            br.setPower(0.0);
        }
    }

    public void tank(double frpower, double flpower, double brpower, double blpower) {
        frdrive.setPower(frpower);
        fldrive.setPower(flpower);
        brdrive.setPower(brpower);
        bldrive.setPower(blpower);
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

    public void doImageL() {
        Log.i("Hi", "JHewre");
    }

    public void doImageC() {
        Log.i("Hi", "JHee");
    }

    public void doImageR() {
        Log.i("Hi", "JHe");
    }

    public void jewelAuto(ColorSensor sensor, Servo servo) {
        servo.setPosition(0.3);
        if(sensor.red() > 11 || sensor.blue() < 2) {
            servo.setPosition(0);
        }
        else if(sensor.blue() > 11 || sensor.red() < 2) {
            servo.setPosition(1);
        }
        else {
            servo.setPosition(0.3);
        }
        telemetry.addData("Red value", sensor.red());
        telemetry.addData("Blue value", sensor.blue());
    }

    public void setupVuforia(int image) {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        Log.i("setup", "workss");
        parameters.vuforiaLicenseKey = V_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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

    public void vuforiaAuto(boolean imageLDetected, boolean imageCDetected, boolean imageRDetected) {
        boolean vu = true;
        trackables.activate();

        while (vu) {
            OpenGLMatrix currentLocation = listener.getUpdatedRobotLocation();

            if (currentLocation != null) {
                lastKnownLocation = currentLocation;
            }

            RelicRecoveryVuMark detector = RelicRecoveryVuMark.from(trackable);

            if (listener.isVisible()) {
                if (detector == RelicRecoveryVuMark.LEFT) {
                    imageLDetected = true;
                    vu = false;
                } else if (detector == RelicRecoveryVuMark.CENTER) {
                    imageCDetected = true;
                    vu = false;
                } else if (detector == RelicRecoveryVuMark.RIGHT) {
                    imageRDetected = true;
                    vu = false;
                }
            }

            telemetry.addData("Image L", imageLDetected);
            telemetry.addData("Image C", imageCDetected);
            telemetry.addData("Image R", imageRDetected);
            telemetry.addData("Image", detector);
            telemetry.addData("Last Known Location ", formatMatrix(lastKnownLocation));

            if (imageLDetected) {
                doImageL();
            }
            if (imageCDetected) {
                doImageC();

            }
            if (imageRDetected) {
                doImageR();
            }
        }
    }
}
