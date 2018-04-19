/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptNullOp;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoBlue;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoBlue2;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoBlue285;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoBlue85;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoRed;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoRed2;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoRed285;
import org.firstinspires.ftc.robotcontroller.internal.Auto.autoRed85;
import org.firstinspires.ftc.robotcontroller.internal.Auto.glyphAutoTest;
import org.firstinspires.ftc.robotcontroller.internal.Auto.jewelAutoBlue;
import org.firstinspires.ftc.robotcontroller.internal.Auto.jewelAutoRed;
import org.firstinspires.ftc.robotcontroller.internal.Tests.distanceSensorTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.drivePIDTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.encoderCountTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.intakeTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.jewelAutoTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.liftTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.revIMUTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.sensorTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.servoTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.teleOpTest;
import org.firstinspires.ftc.robotcontroller.internal.Tests.turnTest;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta.Flavor;

/**
 * {@link FtcOpModeRegister} is responsible for registering opmodes for use in an FTC game.
 * @see #register(OpModeManager)
 */
public class FtcOpModeRegister implements OpModeRegister {

    /**
     * {@link #register(OpModeManager)} is called by the SDK game in order to register
     * OpMode classes or instances that will participate in an FTC game.
     *
     * There are two mechanisms by which an OpMode may be registered.
     *
     *  1) The preferred method is by means of class annotations in the OpMode itself.
     *  See, for example the class annotations in {@link ConceptNullOp}.
     *
     *  2) The other, retired,  method is to modify this {@link #register(OpModeManager)}
     *  method to include explicit calls to OpModeManager.register().
     *  This method of modifying this file directly is discouraged, as it
     *  makes updates to the SDK harder to integrate into your code.
     *
     * @param manager the object which contains methods for carrying out OpMode registrations
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.TeleOp
     * @see com.qualcomm.robotcore.eventloop.opmode.Autonomous
     */
    public void register(OpModeManager manager) {

        /**
         * Any manual OpMode class registrations should go here.
         */

        manager.register(new OpModeMeta("encoder test", Flavor.AUTONOMOUS), encoderCountTest.class);
        manager.register(new OpModeMeta("Jewel Auto Test", Flavor.AUTONOMOUS), jewelAutoTest.class);
        manager.register(new OpModeMeta("Glyph Auto Test", Flavor.AUTONOMOUS), glyphAutoTest.class);
        manager.register(new OpModeMeta("Close Auto Red", Flavor.AUTONOMOUS), autoRed.class);
        manager.register(new OpModeMeta("Close Auto Blue", Flavor.AUTONOMOUS), autoBlue.class);
        manager.register(new OpModeMeta("Far Auto Red", Flavor.AUTONOMOUS), autoRed2.class);
        manager.register(new OpModeMeta("Far Auto Blue", Flavor.AUTONOMOUS), autoBlue2.class);
        manager.register(new OpModeMeta("Close Auto Red 85", Flavor.AUTONOMOUS), autoRed85.class);
        manager.register(new OpModeMeta("Close Auto Blue 85", Flavor.AUTONOMOUS), autoBlue85.class);
        manager.register(new OpModeMeta("Far Auto Red 85", Flavor.AUTONOMOUS), autoRed285.class);
        manager.register(new OpModeMeta("Far Auto Blue 85", Flavor.AUTONOMOUS), autoBlue285.class);
        manager.register(new OpModeMeta("Jewel Auto Blue", Flavor.AUTONOMOUS), jewelAutoBlue.class);
        manager.register(new OpModeMeta("Jewel Auto Red", Flavor.AUTONOMOUS), jewelAutoRed.class);
        manager.register("TeleOp", teleOp.class);
        manager.register("test Tele", teleOpTest.class);
        manager.register("servo test", servoTest.class);
        manager.register("sensor test", sensorTest.class);
        manager.register("Rev IMU test", revIMUTest.class);
        manager.register("intake test", intakeTest.class);
        manager.register("lift test", liftTest.class);
        manager.register("Turn test", turnTest.class);
        manager.register("PID test", drivePIDTest.class);
        manager.register("Distance test", distanceSensorTest.class);
    }
}
