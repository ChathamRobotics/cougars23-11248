package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleopBot;

@TeleOp(name="Basic Drive")
public class BasicDrive extends LinearOpMode
{
    private final TeleopBot robot = new TeleopBot();
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastPowerChangeTime;
    private double clawLiftPower = 1;

    @Override
    public void runOpMode()
    {
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        robot.init(hardwareMap);
        telemetry.speak("Robot inishiated");

        waitForStart();
        robot.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive())
        {
            // Adjust Power
            if (runtime.time()  > lastPowerChangeTime + 0.5)
            {
                if (gamepad1.dpad_down)
                {
                    robot.basePower = Math.max(0, robot.basePower - 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad1.dpad_up)
                {
                    robot.basePower = Math.min(1, robot.basePower + 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                if (gamepad2.dpad_down)
                {
                    clawLiftPower = Math.max(0, clawLiftPower - 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad2.dpad_up)
                {
                    clawLiftPower = Math.min(1, clawLiftPower + 0.1);
                    lastPowerChangeTime = runtime.time();
                }
            }
            if (gamepad2.left_bumper) {
                robot.clawL.setPosition(0);
                robot.clawR.setPosition(1);
            }
            else if (gamepad2.right_bumper) {
                robot.clawL.setPosition(1);
                robot.clawR.setPosition(0);
            }
            if (gamepad2.cross) {
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad2.triangle) {
                robot.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            // Calculate Power
            robot.move(gamepad1.left_stick_y * -1);
            robot.turn(gamepad1.right_stick_x);
            robot.strafe(gamepad1.left_trigger - gamepad1.right_trigger);

            // Actually turn the motors
            robot.move();
            robot.clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int CLAW_LIFT_MAX = 3250;
            double REVERSE_SPOOL_MULTI = 0.77;
            if ((gamepad2.left_stick_y * clawLiftPower * -1 > 0 && robot.clawLift.getCurrentPosition() < CLAW_LIFT_MAX - 50) || (gamepad2.left_stick_y * clawLiftPower * -1 < 0 && robot.clawLift.getCurrentPosition() > 100) || (!(robot.clawLift.getCurrentPosition() <= 100) && gamepad2.left_stick_y != 0 && !(robot.clawLift.getCurrentPosition() >= CLAW_LIFT_MAX - 50))) {
                if (gamepad2.left_stick_y * -1 < 0) {
                    robot.clawLift.setPower(0);
                } else if (gamepad2.left_stick_y != 0) {
                    robot.clawLift.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
                    robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    telemetry.addData("currently moving", "clawLift");
                } else {
                    robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (robot.clawLift.getCurrentPosition() >= 100 && robot.clawLift.getCurrentPosition() <= CLAW_LIFT_MAX - 50) {
                robot.clawLift.setPower(0);
            } else if(robot.clawLift.getCurrentPosition() < 100) {
                if (robot.clawLift.getCurrentPosition() < 0) {
                    robot.clawLift.setPower(clawLiftPower * 0.1);
                } else {
                    robot.clawLift.setPower(clawLiftPower * 0.1 * -1);
                }
                robot.clawLift.setTargetPosition(0);
                robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if(robot.clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 50) {
                if (robot.clawLift.getCurrentPosition() < CLAW_LIFT_MAX) {
                    robot.clawLift.setPower(clawLiftPower * 0.1);
                } else {
                    robot.clawLift.setPower(clawLiftPower * 0.1 * -1);
                }
                robot.clawLift.setTargetPosition(CLAW_LIFT_MAX);
                robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.left_stick_y * -1 < 0) {
                robot.reverseSpool.setPower(clawLiftPower * clawLiftPower * REVERSE_SPOOL_MULTI);
                telemetry.addData("currently moving", "reverseSpool");
            } else if (gamepad2.left_stick_y != 0) {
                robot.reverseSpool.setPower(0);
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            /*
            if (robot.reverseSpool.getCurrentPosition() < robot.clawLift.getCurrentPosition()) {
                robot.reverseSpool.setPower(clawLiftPower * clawLiftPower * REVERSE_SPOOL_MULTI);
            } else {
                robot.reverseSpool.setPower(clawLiftPower * clawLiftPower * REVERSE_SPOOL_MULTI * -1);
            }
            double bias = 0;
            if (gamepad2.left_stick_y * -1 > 0) bias += 50;
            if (gamepad2.left_stick_y * -1 < 0) bias -= 50;
            if (robot.clawLift.getCurrentPosition() < 100 || robot.clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 100) bias *= 0.6;
            if (robot.clawLift.getCurrentPosition() < 50 || robot.clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 50) bias *= 0.2;
             */
           // robot.reverseSpool.setTargetPosition((int)(robot.clawLift.getCurrentPosition() * REVERSE_SPOOL_MULTI + bias));

            robot.clawIntake.setPower(gamepad2.right_stick_y * 0.2 * -1);


            // Display the current motor name, encoder position, and power
            telemetry.addData("Status", "Running");
            telemetry.addData("Power", robot.basePower);
            telemetry.addData("Claw Lift Power", clawLiftPower);
            telemetry.addData("ClawL Servo Position", robot.clawL.getPosition());
            telemetry.addData("ClawR Servo Position", robot.clawR.getPosition());
            telemetry.addData("Claw Lift Position", robot.clawLift.getCurrentPosition());
            telemetry.addData("Reverse Spool Position", robot.reverseSpool.getCurrentPosition());
            telemetry.addData("Reverse Spool Target Position", robot.reverseSpool.getTargetPosition());
            telemetry.addData("A", gamepad2.left_stick_y * clawLiftPower * -1);
            telemetry.update();
        }
    }
}