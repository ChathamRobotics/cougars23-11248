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

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the hardware for our 22-23 robot.
 *
 * */
public class TeleopBot extends BaseBot
{
    private double lfPower = 0;
    private double lbPower = 0;
    private double rfPower = 0;
    private double rbPower = 0;
    private double lfTarget = 0;
    private double lbTarget = 0;
    private double rfTarget = 0;
    private double rbTarget = 0;
    public double basePower = 0.4;

    private final ElapsedTime runtime = new ElapsedTime();
    private double lastTime;


    public void move() {
        leftFront.setPower(lfPower);
        leftBack.setPower(lbPower);
        rightFront.setPower(rfPower);
        rightBack.setPower(rbPower);

        /*
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + (int)lfTarget);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + (int)lbTarget);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + (int)rfTarget);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + (int)rbTarget );
         */


        lfPower = 0;
        lbPower = 0;
        rfPower = 0;
        rbPower = 0;

        lastTime = runtime.milliseconds();
    }

    public void move(double power) {
        lfPower += power * basePower;
        lbPower += power * basePower;
        rfPower += power * basePower;
        rbPower += power * basePower;

        lfTarget = runtime.milliseconds() - lastTime * power;
        lbTarget = runtime.milliseconds() - lastTime * power;
        rfTarget = runtime.milliseconds() - lastTime * power;
        rbTarget = runtime.milliseconds() - lastTime * power;
    }

    public void turn(double power) {
        lfPower += power * basePower;
        lbPower += power * basePower;
        rfPower -= power * basePower;
        rbPower -= power * basePower;

        lfTarget = runtime.milliseconds() - lastTime * power;
        lbTarget = runtime.milliseconds() - lastTime * power;
        rfTarget = (runtime.milliseconds() - lastTime * power) * -1;
        rbTarget = (runtime.milliseconds() - lastTime * power) * -1;
    }

    public void strafe(double power) {
        lfPower += power * basePower;
        lbPower -= power * basePower;
        rfPower -= power * basePower;
        rbPower += power * basePower;

        lfTarget = (runtime.milliseconds() - lastTime * power) * -1;
        lbTarget = runtime.milliseconds() - lastTime * power;
        rfTarget = runtime.milliseconds() - lastTime * power;
        rbTarget = (runtime.milliseconds() - lastTime * power) * -1;
    }

}
