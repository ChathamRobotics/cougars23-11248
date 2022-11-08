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
                if (gamepad1.dpad_left)
                {
                    robot.claw.setPosition(robot.claw.getPosition() + 0.01);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad1.dpad_right)
                {
                    robot.claw.setPosition(robot.claw.getPosition() - 0.01);
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
                robot.claw.setPosition(.20);
            }
            else if (gamepad2.right_bumper) {
                robot.claw.setPosition(.80);
            }
            if (gamepad2.cross) {
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                robot.clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // Calculate Power
            robot.move(gamepad1.left_stick_y * -1);
            robot.turn(gamepad1.right_stick_x);
            robot.strafe(gamepad1.left_trigger - gamepad1.right_trigger);

            // Actually turn the motors
            robot.move();
            if (!(gamepad2.left_stick_y * clawLiftPower * -1 < 0) || (!(robot.clawLift.getCurrentPosition() <= 100) && gamepad2.left_stick_y != 0)) {
                robot.clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.clawLift.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
            } else {
                robot.clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.clawLift.setPower(clawLiftPower);
                robot.clawLift.setTargetPosition(0);
            }
            // Display the current motor name, encoder position, and power
            telemetry.addData("Status", "Running");
            telemetry.addData("Power", robot.basePower);
            telemetry.addData("Claw Lift Power", clawLiftPower);
            telemetry.addData("Claw Servo Position", robot.claw.getPosition());
            telemetry.addData("Claw Lift Position", robot.clawLift.getCurrentPosition());
            telemetry.addData("A", gamepad2.left_stick_y * clawLiftPower * -1);
            telemetry.update();
        }
    }
}