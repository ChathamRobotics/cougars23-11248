package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.CLAW_LIFT_MAX;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.INTAKE_BOTTOM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

public class MecanumDrive extends BaseMecanumDrive {
    private static final double CLAW_LIFT_ERROR = 10;
    private static final double CLAW_LIFT_MAX_VELOCITY = 1;
    private static final double CLAW_LIFT_ACCEL = 200;

    private static final double CLAW_INTAKE_ERROR = 10;
    private static final double CLAW_INTAKE_MAX_VELOCITY = 0.4;
    private static final double CLAW_INTAKE_ACCEL = 300;

    public DcMotorEx clawLift1, clawLift2;
    public List<DcMotorEx> clawLift;
    public DcMotorEx reverseSpool;
    public DcMotorEx clawIntake;

    public Servo clawL, clawR;

    public MecanumDrive (HardwareMap hwMap) {
        super(hwMap);

        clawLift1 = hwMap.get(DcMotorEx.class, "clawLift");
        clawLift2 = hwMap.get(DcMotorEx.class, "clawLift2");
        clawLift = Arrays.asList(clawLift1, clawLift2);

        reverseSpool = hwMap.get(DcMotorEx.class, "reverseSpool");
        clawIntake = hwMap.get(DcMotorEx.class, "clawIntake");

        clawL = hwMap.get(Servo.class, "clawL");
        clawR = hwMap.get(Servo.class, "clawR");

        clawLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        clawLift1.setDirection(DcMotorSimple.Direction.REVERSE);
        clawLift2.setDirection(DcMotorSimple.Direction.FORWARD);
        reverseSpool.setDirection(DcMotorSimple.Direction.FORWARD);
        clawIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        clawLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        reverseSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawLift1.setPower(0);
        clawLift2.setPower(0);
        reverseSpool.setPower(0);
        clawIntake.setPower(0);

        clawL.scaleRange(0.07, 0.21);
        clawR.scaleRange(0.57, 0.71);
    }

    /**
     * Set the claw to a specified position
     * @param position Position to set the claw to - 0 is open, 1 is closed
     * @param waitToPos Whether or not to wait until the claw is at the specified position before going on
     */
    public void setClawPosition(double position, boolean waitToPos) {
        clawL.setPosition(1 - position);
        clawR.setPosition(position);

        if (waitToPos) {
            try {
                Thread.sleep(500);
            } catch(InterruptedException e) {
                System.out.println("got interrupted!");
            }
        }
    }

    /**
     * Function to get the power based on a PID function for a given motor
     * Max power is 1
     * @param current Current position of the motor
     * @param target Target position of the motor
     * @param maxPower Max power (0 - 1)
     * @param accel Time to accelerate (0 - 1/2 of target)
     * @return double representing power (between 0 and 1)
     */
    private double PIDPower(double current, double target, double maxPower, double accel) {
        if (target < 0) {
            target = -target;
            current = -current;
        }
        // PID function
        double PID = -Math.abs( (2 * (current - target/2)) / (2 * accel) ) + target/(2 * accel) + (maxPower / 5);

        // Clamp
        return Math.max(0, Math.min(maxPower, PID)
        );
    }

    /**
     * Sets the claw lift to a specified position using both claw lift motors and the reverse spool
     * @param position Value from 0-1 where 0 is at min height and 1 is at max height
     */
    public void setClawLiftPosition(double position, LinearOpMode opMode) {
        int oldClawLiftPos = clawLift1.getCurrentPosition();
        double target = CLAW_LIFT_MAX * position;


        opMode.telemetry.addData("clawLift1 position", clawLift1.getCurrentPosition());
        opMode.telemetry.addData("target", target);
        opMode.telemetry.addData("math", Math.abs(clawLift1.getCurrentPosition() - target));
        opMode.telemetry.update();
        while (Math.abs(clawLift1.getCurrentPosition() - target) > CLAW_LIFT_ERROR) {
            opMode.telemetry.addData("clawLift1 position", clawLift1.getCurrentPosition());
            opMode.telemetry.addData("target", target);
            double PIDPower = PIDPower(clawLift1.getCurrentPosition() - oldClawLiftPos, target - oldClawLiftPos, CLAW_LIFT_MAX_VELOCITY, CLAW_LIFT_ACCEL);
            opMode.telemetry.addData("PIDPower", PIDPower);
            if (clawLift1.getCurrentPosition() < target) {
                // Go up -> clawLift
                for (DcMotorEx motor : clawLift) {
                    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    motor.setPower(PIDPower);
                }
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                reverseSpool.setPower(0);
                opMode.telemetry.addData("Running", "clawLift");
                opMode.telemetry.addData("clawLift1 Power", clawLift1.getPower());
                opMode.telemetry.addData("clawLift2 Power", clawLift2.getPower());
            }
            else {
                // Go down -> reverseSpool
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                reverseSpool.setPower(PIDPower);
                for (DcMotorEx motor : clawLift) {
                    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    motor.setPower(0);
                }
                opMode.telemetry.addData("Running", "reverseSpool");
            }
            opMode.telemetry.update();
        }
    }

    /**
     * Sets the claw lift to a specified position using both claw lift motors and the reverse spool
     * @param position Value from 0-1 where 0 is at fortnite and 1 is fully out
     */
    public void setClawIntakePosition(double position, LinearOpMode opMode) {
        int oldClawIntakePos = clawIntake.getCurrentPosition();
        double target = INTAKE_BOTTOM * position;


        while (Math.abs(clawIntake.getCurrentPosition() - target) > CLAW_INTAKE_ERROR) {
            opMode.telemetry.addData("Claw Intake Pos", clawIntake.getCurrentPosition());
            opMode.telemetry.addData("Claw Intake Target", target);
            double PIDPower = PIDPower(clawIntake.getCurrentPosition() - oldClawIntakePos, target - oldClawIntakePos, CLAW_INTAKE_MAX_VELOCITY, CLAW_INTAKE_ACCEL);
            if (clawIntake.getCurrentPosition() < target) {
                clawIntake.setPower(PIDPower);
            } else {
                clawIntake.setPower(-PIDPower);
            }
            opMode.telemetry.addData("PIDPower", PIDPower);
            opMode.telemetry.addData("Claw Intake Power", clawIntake.getPower());
            opMode.telemetry.addData("PIDPower 1", clawIntake.getCurrentPosition() - oldClawIntakePos);
            opMode.telemetry.addData("PIDPower 2", target - oldClawIntakePos);
            opMode.telemetry.addData("math", Math.abs(clawIntake.getCurrentPosition() - target));
            opMode.telemetry.update();
        }
    }
}
