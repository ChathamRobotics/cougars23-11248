package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_LIFT_MAX;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_BOTTOM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.TeleopBot;

@TeleOp(name="Adv Drive")
public class AdvDrive extends LinearOpMode
{
    private final TeleopBot robot = new TeleopBot();
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastPowerChangeTime;
    private boolean boost = false;
    private double clawLiftPower = 1;
    private double clawIntakePower = 0.3;
    private int runningMacro = 0;
    private double macroTime = 0;
    final double REVERSE_SPOOL_MULTI = 0.77;

    @Override
    public void runOpMode()
    {
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        robot.init(hardwareMap);
        telemetry.speak("Robot inishiated");

        waitForStart();
        robot.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.clawLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive())
        {
            // Fine Adjust Power
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

            // Quick Adjust Power
            if (gamepad1.y) {
                robot.basePower = 0.8;
            } else if (gamepad1.a) {
                robot.basePower = 0.4;
            }
            if (gamepad1.left_bumper) {
                robot.basePower = 0.9;
                boost = true;
            } else if (gamepad1.right_bumper) {
                robot.basePower = 0.2;
                boost = true;
            } else if (!gamepad1.right_bumper && !gamepad1.left_bumper && boost) {
                robot.basePower = 0.4;
                boost = false;
            }


            // Claw
            if (gamepad2.left_bumper) {
                // open
                robot.clawL.setPosition(1);
                robot.clawR.setPosition(0);
            }
            else if (gamepad2.right_bumper) {
                // close
                robot.clawL.setPosition(0);
                robot.clawR.setPosition(1);
            }
            else if (gamepad2.right_trigger > 0) {
                // slightly more closed than halfway
                robot.clawL.setPosition(0.25);
                robot.clawR.setPosition(0.75);
            }

            // Clawlift Slowdown & Brake
            if (gamepad2.left_trigger > 0) {
                clawIntakePower = 0.075;
            } else {
                clawIntakePower = 0.3;
            }

            if (gamepad2.cross) {
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // ClawLift reset encoders
            if (gamepad2.triangle) { // AKA Y
                robot.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.clawLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.reverseSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.circle) { // AKA B
                robot.clawIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Macros

            // Pick cone from substation; Place on tall junction
            if (gamepad2.square || (runningMacro >= 100 && runningMacro < 200)) {
                if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                    runningMacro = 0;
                    continue;
                }
                telemetry.addLine("Macro Activated: Pick and Place - Substation -> Tall Junction");
                //telemetry.addData("Macro", runningMacro);
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.clawIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                switch (runningMacro) {
                    case 0:
                    case 100:
                        runningMacro = 100;
                        telemetry.addLine("Status: Ensuring start position");
                        telemetry.addData("ClawLift pos", robot.clawLift.getCurrentPosition());
                        telemetry.addData("ClawIntake pos", robot.clawIntake.getCurrentPosition());
                        telemetry.addData("ClawIntake power", robot.clawIntake.getPower());
                        telemetry.addData("test", (double)robot.clawIntake.getCurrentPosition());
                        telemetry.addData("test", Math.PI * (double)robot.clawIntake.getCurrentPosition() / (double)CLAW_LIFT_MAX);
                        robot.clawLift.setPower(0);
                        robot.clawLift2.setPower(0);
                        robot.reverseSpool.setPower(-1);
                        robot.clawIntake.setPower((-0.8 * Math.sin(Math.PI * robot.clawIntake.getCurrentPosition() / CLAW_LIFT_MAX)) - 0.2);
                        robot.clawLift.setTargetPosition(0);
                        robot.clawLift2.setTargetPosition(0);
                        robot.reverseSpool.setTargetPosition(0);
                        robot.clawIntake.setTargetPosition(INTAKE_BOTTOM);
                        robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.clawLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.reverseSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.clawIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.clawL.setPosition(0);
                        robot.clawR.setPosition(1);
                        if (robot.clawLift.getCurrentPosition() <= 0 && robot.clawIntake.getCurrentPosition() <= INTAKE_BOTTOM && robot.reverseSpool.getCurrentPosition() >= 0) {
                            runningMacro = 105;
                            macroTime = runtime.milliseconds();
                        }
                        break;
                    case 105:
                        telemetry.addLine("Status: Closing Claw");
                        telemetry.addData("Claw Pos", robot.clawL.getPosition());
                        robot.clawL.setPosition(1);
                        robot.clawR.setPosition(0);
                        if (runtime.milliseconds() > macroTime + 1500) runningMacro = 110;
                        break;
                    case 110:
                        telemetry.addLine("Status: Picking up cone and going up");
                        telemetry.addData("ClawLift pos", robot.clawLift.getCurrentPosition());
                        telemetry.addData("ClawIntake pos", robot.clawIntake.getCurrentPosition());
                        robot.clawLift.setPower(1);
                        robot.clawLift2.setPower(1);
                        robot.reverseSpool.setPower(0);
                        robot.clawIntake.setPower(0.5 * Math.sin(Math.PI * robot.clawIntake.getCurrentPosition() / CLAW_LIFT_MAX) + 0.1);
                        robot.clawLift.setTargetPosition(CLAW_LIFT_MAX);
                        robot.clawLift2.setTargetPosition(CLAW_LIFT_MAX);
                        robot.reverseSpool.setTargetPosition(-CLAW_LIFT_MAX);
                        robot.clawIntake.setTargetPosition(0);
                        if (robot.clawLift.getCurrentPosition() >= CLAW_LIFT_MAX && robot.clawIntake.getCurrentPosition() >= 0) {
                            runningMacro = 115;
                            macroTime = runtime.milliseconds();
                        }
                        break;
                    case 115:
                        telemetry.addLine("Status: Opening Claw");
                        telemetry.addData("Claw Pos", robot.clawL.getPosition());
                        robot.clawL.setPosition(0);
                        robot.clawR.setPosition(1);
                        if (runtime.milliseconds() > macroTime + 1500) runningMacro = 110;
                        break;
                }

                telemetry.update();
                continue;
            }


            // Calculate Power
            robot.move(gamepad1.left_stick_y * -1);
            robot.turn(gamepad1.right_stick_x);
            double leftY = gamepad1.left_stick_x;
            if (Math.abs(leftY) < 0.8) leftY = 0;
            robot.strafe(leftY + gamepad1.right_trigger - gamepad1.left_trigger);
            //robot.strafe(gamepad1.left_trigger - gamepad1.right_trigger);

            // Actually turn the motors
            robot.move();


            robot.clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if ((gamepad2.left_stick_y * clawLiftPower * -1 > 0 && robot.clawLift.getCurrentPosition() < CLAW_LIFT_MAX - 50) || (gamepad2.left_stick_y * clawLiftPower * -1 < 0 && robot.clawLift.getCurrentPosition() > 100) || (!(robot.clawLift.getCurrentPosition() <= 100) && gamepad2.left_stick_y != 0 && !(robot.clawLift.getCurrentPosition() >= CLAW_LIFT_MAX - 50))) {
                if (gamepad2.left_stick_y * -1 < 0) {
                    robot.clawLift.setPower(0);
                    robot.clawLift2.setPower(0);
                } else if (gamepad2.left_stick_y != 0) {
                    robot.clawLift.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
                    robot.clawLift2.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
                    robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    telemetry.addData("currently moving", "clawLift");
                } else {
                    robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (robot.clawLift.getCurrentPosition() >= 100 && robot.clawLift.getCurrentPosition() <= CLAW_LIFT_MAX - 50) {
                robot.clawLift.setPower(0);
                robot.clawLift2.setPower(0);
            } else if(robot.clawLift.getCurrentPosition() < 100) {
                if (robot.clawLift.getCurrentPosition() < 0) {
                    robot.clawLift.setPower(clawLiftPower * 0.1);
                    robot.clawLift2.setPower(clawLiftPower * 0.1);
                } else {
                    robot.clawLift.setPower(clawLiftPower * 0.1 * -1);
                    robot.clawLift2.setPower(clawLiftPower * 0.1 * -1);
                }
                robot.clawLift.setTargetPosition(0);
                robot.clawLift2.setTargetPosition(0);
                robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.clawLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if(robot.clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 50) {
                if (robot.clawLift.getCurrentPosition() < CLAW_LIFT_MAX) {
                    robot.clawLift.setPower(clawLiftPower * 0.1);
                    robot.clawLift2.setPower(clawLiftPower * 0.1);
                } else {
                    robot.clawLift.setPower(clawLiftPower * 0.1 * -1);
                    robot.clawLift2.setPower(clawLiftPower * 0.1 * -1);
                }
                robot.clawLift.setTargetPosition(CLAW_LIFT_MAX);
                robot.clawLift2.setTargetPosition(CLAW_LIFT_MAX);
                robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.clawLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.left_stick_y * -1 < 0) {
                robot.reverseSpool.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
                //robot.clawLift.setPower(gamepad2.left_stick_y * clawLiftPower * REVERSE_SPOOL_MULTI * -1);
                //robot.clawLift2.setPower(gamepad2.left_stick_y * clawLiftPower * REVERSE_SPOOL_MULTI * -1);
                telemetry.addData("currently moving", "reverseSpool");
            } else if (gamepad2.left_stick_y != 0) {
                robot.reverseSpool.setPower(0);
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                robot.reverseSpool.setPower(0);
                robot.reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            /*
            if (robot.reverseSpool.getCurrentPosition() < robot.clawLift.getCurrentPosition()) {
                robot.reverseSpool.setPower(clawLiftPower * clawLiftPower * REVERSE_SPOOL_MULTI * -1);
            } else {
                robot.reverseSpool.setPower(clawLiftPower * clawLiftPower * REVERSE_SPOOL_MULTI);
            }
            double bias = 0;
            if (gamepad2.left_stick_y * -1 > 0) bias += 50;
            if (gamepad2.left_stick_y * -1 < 0) bias -= 50;
            if (robot.clawLift.getCurrentPosition() < 100 || robot.clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 100) bias *= 0.6;
            if (robot.clawLift.getCurrentPosition() < 50 || robot.clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 50) bias *= 0.2;
             */
           // robot.reverseSpool.setTargetPosition((int)(robot.clawLift.getCurrentPosition() * REVERSE_SPOOL_MULTI + bias));

            if ((gamepad2.right_stick_y * -1 < 0 && robot.clawIntake.getCurrentPosition() > INTAKE_BOTTOM) || (gamepad2.right_stick_y * -1 > 0 && robot.clawIntake.getCurrentPosition() < 0)) {
                robot.clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (Math.abs(gamepad2.right_stick_y) > 0.99) {
                    robot.clawIntake.setPower(gamepad2.right_stick_y * clawIntakePower * - 1);
                }
                else {
                    robot.clawIntake.setPower(gamepad2.right_stick_y * clawIntakePower * - 1 * 0.2);
                }
            } else if (robot.clawIntake.getCurrentPosition() > 0) {
                robot.clawIntake.setPower(-0.05);
                robot.clawIntake.setTargetPosition(0);
                robot.clawIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                robot.clawIntake.setPower(0);
            }

            // Display the current motor name, encoder position, and power
            telemetry.addData("Status", "Running");
            //telemetry.addData("Ultrasonic", robot.ultrasonic.getUltrasonicLevel());
            telemetry.addData("Power", robot.basePower);
            telemetry.addData("Claw Lift Power", clawLiftPower);
            telemetry.addData("ClawL Servo Position", robot.clawL.getPosition());
            telemetry.addData("ClawR Servo Position", robot.clawR.getPosition());
            telemetry.addData("Claw Lift Position", robot.clawLift.getCurrentPosition());
            telemetry.addData("Claw Lift Power", robot.clawLift.getPower());
            telemetry.addData("Reverse Spool Position", robot.reverseSpool.getCurrentPosition());
            telemetry.addData("Reverse Spool Target Position", robot.reverseSpool.getTargetPosition());
            telemetry.addData("Reverse Spool Power", robot.reverseSpool.getPower());
            telemetry.addData("A", gamepad2.right_stick_y);
            telemetry.addData("Claw Intake Pos", robot.clawIntake.getCurrentPosition());
            telemetry.addData("Claw Intake Power", clawIntakePower);
            telemetry.addData("Claw Intake Speed", robot.clawIntake.getPower());
            telemetry.addData("Macro", runningMacro);
            telemetry.update();
        }
    }
}