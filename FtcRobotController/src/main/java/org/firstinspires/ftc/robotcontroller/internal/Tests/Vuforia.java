package org.firstinspires.ftc.robotcontroller.internal.Tests;


import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
 * Created by sahith on 8/27/17.
 */
public class Vuforia extends LinearOpMode{

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables trackables;
    VuforiaTrackable trackable;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String V_KEY = "Aelee1z/////AAAAGSOzCHm55k0ymtB98J5uxqsJ3VZ0L7SyL2P9mkwAcASPo5jphdOuL+kEucaxbqfA3GfMKZbO9/7zChpv5oYcZ+yc/T6cl7LjsJVETcd2kf+6W1cby1xBzFBwuHi0wHKvxx+PmsKhwA0dLReiaVIkev7aJ8CrAhDfPfkeT8HMXNRvoJ3tkiAkfc9ONcNxNt4XzbXTcslg8/+xgHw/q7yhnuWy1hgPizTCcIQr+oWRuukFDC228GuWhTiHDGFqHWMd0dublyymuEgSxpSByagvaHJ73/mJbfRCTcBJgmvLmAzv86N6vyycC4pReQxHn8djz8QCicZnEbz9CM10HcaRxqftMFNv0iKnlDH85NJrnyMw";

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

    @Override
    public void runOpMode() throws InterruptedException {
        boolean imageLDetected = false, imageCDetected = false, imageRDetected = false;

        setupVuforia(0);
        Log.i("test", "workss");

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);             // Coordinates are in millimeters and are based off of the center of the robot

        waitForStart();

        trackables.activate();

        while (opModeIsActive()) {
            OpenGLMatrix currentLocation = listener.getUpdatedRobotLocation();

            if(currentLocation != null) {
                lastKnownLocation = currentLocation;
            }

            RelicRecoveryVuMark detector = RelicRecoveryVuMark.from(trackable);

            if (listener.isVisible()) {
                if (detector == RelicRecoveryVuMark.LEFT) {
                    imageLDetected = true;
                }
                else if (detector == RelicRecoveryVuMark.CENTER) {
                    imageCDetected = true;
                }
                else if (detector == RelicRecoveryVuMark.RIGHT) {
                    imageRDetected = true;
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

            telemetry.update();
        }
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
}
