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

package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the hardware for our 2021-2022 robot.
 *
 * */
public abstract class BaseBot
{
    /** Encoder counts per motor revolution, usually found in the motor's datasheet */
    public static final double COUNTS_PER_MOTOR_REV = 28;    // REV Ultraplanetary
    /**
     * The gear ratio for the motor, in this case for two of the REV Ultraplanetary gearboxes
     * Notice that they are weird numbers, despite the gearboxes having integer ratios listed on them
     * ALWAYS check the gearbox datasheet for the true gear ratios, just in case
     * */
    public static final double DRIVE_GEAR_REDUCTION = 2.89 * 3.61;
    /** Wheel diameter in inches, try to be as precise as possible */
    public static final double WHEEL_DIAMETER_INCHES = 3.54330709;
    /**
     * Encoder counts per inch the robot moves
     * This is what's actually used to calculate how much the motors should turn
     */
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) * -1; // Negated because the encoders are backwards for some reason

    /*
     * Defining the motors and servos
     * By defining them as variables in this class, we can use them throughout our project,
     * with autocomplete, without having to know the string device names
     */
    public DcMotor      leftFront           = null;
    public DcMotor      leftBack            = null;
    public DcMotor      rightFront          = null;
    public DcMotor      rightBack           = null;
    public DcMotor      clawLift            = null;
    public DcMotor      clawLift2           = null;
    public DcMotor      clawIntake          = null;
    public DcMotor      reverseSpool        = null;
    public Servo        clawL               = null;
    public Servo        clawR               = null;
    public ColorSensor  colorSensorL        = null;
    public ColorSensor  colorSensorR        = null;
    public ModernRoboticsI2cRangeSensor ultrasonic      = null;


    /**
     * Initialize all the hardware, and set static settings like motor direction and running with encoders
     * @param hwMap The HardwareMap to use to initialize the robot
     */
    public void init(HardwareMap hwMap) {
        // Define Motors
        leftFront           = hwMap.get(DcMotor.class, "leftFront");
        leftBack            = hwMap.get(DcMotor.class, "leftBack");
        rightFront          = hwMap.get(DcMotor.class, "rightFront");
        rightBack           = hwMap.get(DcMotor.class, "rightBack");
        clawLift            = hwMap.get(DcMotor.class, "clawLift");
        clawLift2           = hwMap.get(DcMotor.class, "clawLift2");
        clawIntake          = hwMap.get(DcMotor.class, "clawIntake");
        reverseSpool        = hwMap.get(DcMotor.class, "reverseSpool");

        clawL               = hwMap.get(Servo.class, "clawL");
        clawR               = hwMap.get(Servo.class, "clawR");


        ultrasonic          = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic");

        //colorSensorL        = hwMap.get(ColorSensor.class, "colorSensorL");
        //colorSensorR        = hwMap.get(ColorSensor.class, "colorSensorR");


        // Initialize Motor Direction
        leftFront.setDirection(Direction.FORWARD);
        leftBack.setDirection(Direction.FORWARD);
        rightFront.setDirection(Direction.FORWARD);
        rightBack.setDirection(Direction.REVERSE);
        clawLift.setDirection(Direction.REVERSE);
        clawLift2.setDirection(Direction.FORWARD);
        clawIntake.setDirection(Direction.REVERSE);
        reverseSpool.setDirection(Direction.FORWARD);


        // Make sure motors brake on power 0
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        clawLift.setPower(0);
        clawLift2.setPower(0);
        clawIntake.setPower(0);
        reverseSpool.setPower(0);


        // Reset all encoders
        leftFront.setMode(RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(RunMode.STOP_AND_RESET_ENCODER);
        clawLift.setMode(RunMode.STOP_AND_RESET_ENCODER);
        clawLift2.setMode(RunMode.STOP_AND_RESET_ENCODER);
        clawIntake.setMode(RunMode.STOP_AND_RESET_ENCODER);
        reverseSpool.setMode(RunMode.STOP_AND_RESET_ENCODER);


        // Set motors to run with encoder
        leftFront.setMode(RunMode.RUN_USING_ENCODER);
        leftBack.setMode(RunMode.RUN_USING_ENCODER);
        rightFront.setMode(RunMode.RUN_USING_ENCODER);
        rightBack.setMode(RunMode.RUN_USING_ENCODER);
        clawLift.setMode(RunMode.RUN_USING_ENCODER);
        clawLift2.setMode(RunMode.RUN_USING_ENCODER);
        clawIntake.setMode(RunMode.RUN_USING_ENCODER);
        reverseSpool.setMode(RunMode.RUN_USING_ENCODER);


        // Set min and max for servosw
        clawL.scaleRange(0.09, 0.29);
        clawR.scaleRange(0.52, 0.7);
    }

}
