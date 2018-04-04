package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by sahith on 4/2/18.
 */

public class Pseudopad {
    String autoLine;

    public Pseudopad(String line) {
        this.autoLine = line;
    }

    public void readLine() {
        //inverted version of toState()
    }

    public String toState(Gamepad gp) {
        String output = "";
        int temp = 0;
        if (gp.a) temp += (1 << 0);
        if (gp.b) temp += (1 << 1);
        if (gp.x) temp += (1 << 2);
        if (gp.y) temp += (1 << 3);
        if (gp.dpad_down) temp += (1 << 4);
        if (gp.dpad_up) temp += (1 << 5);
        if (gp.left_bumper) temp += (1 << 6);
        if (gp.left_trigger > 0.1) temp += (1 << 7);
        if (gp.right_bumper) temp += (1 << 8);
        if (gp.right_trigger > 0.1) temp += (1 << 9);
        output += temp;

        while (output.length() < 5) output = "0" + output;

        output += doubleToChar(gp.left_stick_x);
        output += doubleToChar(gp.left_stick_y);
        output += doubleToChar(gp.right_stick_x);
        output += doubleToChar(gp.right_stick_y);

        return output;
    }

    public char doubleToChar(double d) {
        for (int i = 1; i <= 26; i++) {
            if (d <= i / 13. - 1) {
                return (char) ('@' + i);
            }
        }
        return 'A';
    }
}
