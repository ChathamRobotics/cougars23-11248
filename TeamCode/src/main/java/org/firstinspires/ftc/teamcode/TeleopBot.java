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
    public double basePower = 0.5;


    public void move() {
        final double lfMult = 1;
        final double lbMult = 0.998558039;
        final double rfMult = 0.995686556;
        final double rbMult = 0.976727786;

        leftFront.setPower(lfPower * lfMult);
        leftBack.setPower(lbPower * lbMult);
        rightFront.setPower(rfPower * rfMult);
        rightBack.setPower(rbPower * rbMult);

        lfPower = 0;
        lbPower = 0;
        rfPower = 0;
        rbPower = 0;
    }

    public void move(double power) {
        lfPower += power * basePower;
        lbPower += power * basePower;
        rfPower += power * basePower;
        rbPower += power * basePower;
    }

    public void turn(double power) {
        lfPower += power * basePower;
        lbPower += power * basePower;
        rfPower -= power * basePower;
        rbPower -= power * basePower;
    }

    public void strafe(double power) {
        lfPower -= power * basePower;
        lbPower += power * basePower;
        rfPower += power * basePower;
        rbPower -= power * basePower;
    }

}
