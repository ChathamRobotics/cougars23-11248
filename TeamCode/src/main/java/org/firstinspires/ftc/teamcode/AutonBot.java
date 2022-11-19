/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the hardware for auton drive.
 *
 * */
public class AutonBot extends BaseBot
{
    public OpenCvWebcam webcam;

    public double basePower = 0.5;

    public Movement movement;

    @Override
    public void init(HardwareMap hwMap) {
        super.init(hwMap);
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        webcam.setMillisecondsPermissionTimeout(2500);

        leftFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightBack.setTargetPosition(0);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void move() {
        if (movement != null) {
            movement.frame(this);

            leftFront.setPower(movement.lfPower);
            leftBack.setPower(movement.lbPower);
            rightFront.setPower(movement.rfPower);
            rightBack.setPower(movement.rbPower);


            leftFront.setTargetPosition((int)movement.expectedLf);
            leftBack.setTargetPosition((int)movement.expectedLb);
            rightFront.setTargetPosition((int)movement.expectedRf);
            rightBack.setTargetPosition((int)movement.expectedRb);


            if (movement.runtime != null && movement.runtime.time() > movement.moveTime) {
                movement = null;
            }
        } else {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

    }

    /**
     * Add a command to the command array <br/>
     * Can be: move, turn, strafe, delay, moveClaw, claw, or switch <br/>
     * - For move and strafe, distance is in inches <br/>
     * - For turn, distance is in degrees <br/>
     * - For delay, distance is not used <br/>
     * - For claw, distance is either 1 for open or 0 for closed <br/>
     * - For moveClaw, distance is 0 for ground, 1 for max height, -1 for short junction, -2 for medium junction, and -3 for tall junction (1900) <br/>
     * - For switch,
     */
    public void command(double time, double distance, String command, LinearOpMode opMode) {
        final ArrayList<String> commands = new ArrayList<>();
        Collections.addAll(commands, "move", "turn", "strafe", "delay", "moveClaw", "claw");
        if (!commands.contains(command)) return;
        movement = new Movement(command, time, distance, 0.4, this);
        movement.init(this);
        while(movement != null) {
            move();

            opMode.telemetry.addData("enc counts per in", Constants.ENCODER_COUNTS_PER_IN);
            opMode.telemetry.addData("lf Motor Encoder", leftFront.getCurrentPosition());
            opMode.telemetry.addData("rf Motor Encoder", rightFront.getCurrentPosition());
            opMode.telemetry.addData("lb Motor Encoder", leftBack.getCurrentPosition());
            opMode.telemetry.addData("rb Motor Encoder", rightBack.getCurrentPosition());
            if (movement != null && movement.runtime != null) {
                opMode.telemetry.addData("lf Expected Pos", movement.expectedLf);
                opMode.telemetry.addData("rf Expected Pos", movement.expectedRf);
                opMode.telemetry.addData("lb Expected Pos", movement.expectedLb);
                opMode.telemetry.addData("rb Expected Pos", movement.expectedRb);
                opMode.telemetry.addData("lf Power", movement.lfPower);
                opMode.telemetry.addData("rf Power", movement.rfPower);
                opMode.telemetry.addData("lb Power", movement.lbPower);
                opMode.telemetry.addData("rb Power", movement.rbPower);
                opMode.telemetry.addData("movement type", movement.type);
                opMode.telemetry.addData("movement progress", movement.runtime.time());
                opMode.telemetry.addData("movement time", movement.moveTime);
                opMode.telemetry.addData("max power", movement.maxPower);
                opMode.telemetry.addData("calc power (lf)", Math.max(Math.min(Math.pow(movement.expectedLf - leftFront.getCurrentPosition(), 1d/3d) / 14d, movement.maxPower), movement.maxPower * -1));
                opMode.telemetry.addData("fps", movement.frameCount / movement.runtime.time());
            }
            opMode.telemetry.update();
        }
    }
}
