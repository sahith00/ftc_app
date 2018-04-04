package org.firstinspires.ftc.robotcontroller.internal.Tests;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 * Created by sahith on 3/24/18.
 */

public class recordAndPlayAutoTest extends LinearOpMode{

    private ElapsedTime timer;
    private BufferedReader br;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        try {
            br = new BufferedReader(new FileReader(
                    new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS),
                            "blue_left.txt")));

            String curr = br.readLine();
            timer.reset();
            while (curr != null) {
                while (timer.seconds() < 30./(58*58)*((curr.charAt(9)-'A')*58+(curr.charAt(10)-'A')));

                curr = br.readLine();
            }
        } catch (IOException ioe) {
            ioe.printStackTrace();
        } catch (NullPointerException npe) {
            npe.printStackTrace();
        } finally {
            try {
                if (br != null) br.close();
            } catch (IOException ioe) {}
        }
    }
}
