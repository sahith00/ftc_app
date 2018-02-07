package org.firstinspires.ftc.robotcontroller.internal.Subsystems;


import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
public class Vuforia {

    public VuforiaLocalizer vuforiaLocalizer;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackables trackables;
    public VuforiaTrackable trackable;
    public VuforiaTrackableDefaultListener listener;

    public OpenGLMatrix lastKnownLocation;
    public OpenGLMatrix phoneLocation;

    public Vuforia(int image, String key) {
        this.parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        this.parameters.vuforiaLicenseKey = key;
        this.parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(this.parameters);
        this.trackables = this.vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        this.trackable = this.trackables.get(image);
        this.trackable.setName("Relic Images");
        this.trackable.setLocation(createMatrix(0, 500, 0, 0, 0, 0));

        this.phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        this.listener = (VuforiaTrackableDefaultListener) this.trackable.getListener();
        this.listener.setPhoneInformation(this.phoneLocation, this.parameters.cameraDirection);

        this.lastKnownLocation = this.createMatrix(0, 0, 0, 0, 0, 0);
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
}