package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Training Wheels")
public class TrainingWheels extends LinearOpMode
{
    private final TeleopBot robot = new TeleopBot();
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastPowerChangeTime;
    private double clawLiftPower = 0.3;

    @Override
    public void runOpMode()
    {
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        robot.init(hardwareMap);
        telemetry.speak("Robot inishiated");
        robot.basePower = 0.2;

        waitForStart();
        robot.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive())
        {
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
            } else {
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad2.circle) {
                robot.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            // Calculate Power
            robot.move(gamepad1.left_stick_y * -1);
            robot.turn(gamepad1.right_stick_x);
            robot.strafe(gamepad1.left_trigger - gamepad1.right_trigger);

            // Actually turn the motors
            robot.move();
            robot.clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if ((gamepad2.left_stick_y * clawLiftPower * -1 > 0 && robot.clawLift.getCurrentPosition() < 3450) || (gamepad2.left_stick_y * clawLiftPower * -1 < 0 && robot.clawLift.getCurrentPosition() > 100) || (!(robot.clawLift.getCurrentPosition() <= 100) && gamepad2.left_stick_y != 0 && !(robot.clawLift.getCurrentPosition() >= 3450))) {
                robot.clawLift.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
            } else if (robot.clawLift.getCurrentPosition() >= 100 && robot.clawLift.getCurrentPosition() <= 3450) {
                robot.clawLift.setPower(0);
            } else if(robot.clawLift.getCurrentPosition() < 100) {
                if (robot.clawLift.getCurrentPosition() < 0) {
                    robot.clawLift.setPower(clawLiftPower * 0.1);
                } else {
                    robot.clawLift.setPower(clawLiftPower * 0.1 * -1);
                }
                robot.clawLift.setTargetPosition(0);
                robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if(robot.clawLift.getCurrentPosition() > 3450) {
                if (robot.clawLift.getCurrentPosition() < 3500) {
                    robot.clawLift.setPower(clawLiftPower * 0.1);
                } else {
                    robot.clawLift.setPower(clawLiftPower * 0.1 * -1);
                }
                robot.clawLift.setTargetPosition(3500);
                robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // Display the current motor name, encoder position, and power
            telemetry.addData("Status", "Running");
            telemetry.addData("Power", robot.basePower);
            telemetry.addData("Claw Lift Power", clawLiftPower);
            telemetry.addData("ClawL Servo Position", robot.clawL.getPosition());
            telemetry.addData("ClawR Servo Position", robot.clawR.getPosition());
            telemetry.addData("Claw Lift Position", robot.clawLift.getCurrentPosition());
            telemetry.addData("A", gamepad2.left_stick_y * clawLiftPower * -1);
            telemetry.update();
        }
    }
}