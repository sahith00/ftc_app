package org.firstinspires.ftc.robotcontroller.internal.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by sahith on 2/6/18.
 */

public class Cipher {
    public ColorSensor glyphColor;
    public int current_glyph;
    public int[][] current_cipher = {
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
    };

    final static int[][] g_snake = {
            {2, 1, 1},
            {2, 2, 1},
            {1, 2, 2},
            {1, 1, 2}
    };
    final static int[][] g_bird = {
            {1, 2, 1},
            {2, 1, 2},
            {2, 1, 2},
            {1, 2, 1}
    };
    final static int[][] g_frog = {
            {2, 1, 2},
            {1, 2, 1},
            {2, 1, 2},
            {1, 2, 1}
    };
    final static int[][] w_snake = {
            {1, 2, 2},
            {1, 1, 2},
            {2, 1, 1},
            {2, 2, 1}
    };
    final static int[][] w_bird = {
            {2, 1, 2},
            {1, 2, 1},
            {1, 2, 1},
            {2, 1, 2}
    };
    final static int[][] w_frog = {
            {1, 2, 1},
            {2, 1, 2},
            {1, 2, 1},
            {2, 1, 2}
    };

    public Cipher(HardwareMap hardwareMap) {
        this.glyphColor = hardwareMap.colorSensor.get("glyphColorSensor");
    }

    public int getGlyphColor() {
        //color sensor detects color of glyph and sets it as the current glyph
        return current_glyph;
    }

    public int[][] placeGlyph(String column) {
        if (column.equals("L")) {
            for (int i = 3; i >= 0; i--) {
                if (current_cipher[i][0] == 0) {
                    current_cipher[i][0] = current_glyph;
                    break;
                }
            }
        }
        if (column.equals("C")) {
            for (int i = 3; i >= 0; i--) {
                if (current_cipher[i][1] == 0) {
                    current_cipher[i][1] = current_glyph;
                    break;
                }
            }
        }
        if (column.equals("R")) {
            for (int i = 3; i >= 0; i--) {
                if (current_cipher[i][2] == 0) {
                    current_cipher[i][2] = current_glyph;
                    break;
                }
            }
        }
        return current_cipher;
    }

    public boolean checkForCipher(int[][] pattern) {
        for (int i = 3; i >= 0; i--) {
            for (int j = 0; j < 3; j++) {
                if (current_cipher[i][0] == 0 && current_cipher[i][1] == 0 && current_cipher[i][2] == 0) {
                    return true;
                }
                if (current_cipher[i][j] != 0 && current_cipher[i][j] != pattern[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }

    public boolean[] checkAllCiphers() {
        boolean[] ciphersWork = {false, false, false, false, false, false};
        ciphersWork[0] = checkForCipher(g_snake);
        ciphersWork[1] = checkForCipher(g_bird);
        ciphersWork[2] = checkForCipher(g_frog);
        ciphersWork[3] = checkForCipher(w_snake);
        ciphersWork[4] = checkForCipher(w_bird);
        ciphersWork[5] = checkForCipher(w_frog);
        return ciphersWork;
    }
}