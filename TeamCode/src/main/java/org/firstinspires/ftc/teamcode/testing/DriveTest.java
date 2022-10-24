package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleopBot;

@TeleOp(name="Drive Test")
public class DriveTest extends LinearOpMode
{
    private final TeleopBot robot = new TeleopBot();
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastPowerChangeTime;
    private double clawLiftPower = 0.5d;

    @Override
    public void runOpMode()
    {
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        robot.init(hardwareMap);

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
                robot.claw.setPosition(0);
            }
            else if (gamepad2.right_bumper) {
                robot.claw.setPosition(1);
            }

            // Calculate Power
            robot.move(gamepad1.left_stick_y);
            robot.turn(gamepad1.right_stick_x);
            robot.strafe(gamepad1.right_trigger - gamepad1.left_trigger);

            // Actually turn the motors
            robot.move();
            robot.clawLift.setPower(gamepad2.left_stick_y * clawLiftPower);

            // Display the current motor name, encoder position, and power
            telemetry.addData("Status", "Running");
            telemetry.addData("Power", robot.basePower);
            telemetry.addData("Claw Lift Power", clawLiftPower);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("lf Motor Encoder", robot.leftFront.getCurrentPosition());
            telemetry.addData("rf Motor Encoder", robot.rightFront.getCurrentPosition());
            telemetry.addData("lb Motor Encoder", robot.leftBack.getCurrentPosition());
            telemetry.addData("rb Motor Encoder", robot.rightBack.getCurrentPosition());
            telemetry.update();
        }
    }
}