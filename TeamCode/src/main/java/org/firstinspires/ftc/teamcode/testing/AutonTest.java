package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonBot;

/**
 * Op mode for testing the connection and direction of motors
 * Use dpad up and down to adjust power and left and right to select a motor
 * Move the left joystick up and down to run the motor
 * NOTE: This does not respect the motor directions set in OurRobot, all motors are set to run forwards
 */

@TeleOp(name="Auton Test")
public class AutonTest extends LinearOpMode
{
    private final AutonBot robot = new AutonBot();
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastInput;

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
            if (runtime.time()  > lastInput + 1)
            {
                if (gamepad1.cross)
                {
                    robot.addMovement(3, 500, "move");
                    lastInput = runtime.time();
                }
                if (gamepad1.triangle)
                {
                    robot.addMovement(3, 500, "strafe");
                    lastInput = runtime.time();
                }
                if (gamepad1.square)
                {
                    robot.addMovement(10, 500, "turn");
                    lastInput = runtime.time();
                }
            }

            robot.move();

            // Display the current motor name, encoder position, and power
            telemetry.addData("Status", "Running");
            telemetry.addData("Power", robot.movement);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("lf Motor Encoder", robot.leftFront.getCurrentPosition());
            telemetry.addData("rf Motor Encoder", robot.rightFront.getCurrentPosition());
            telemetry.addData("lb Motor Encoder", robot.leftBack.getCurrentPosition());
            telemetry.addData("rb Motor Encoder", robot.rightBack.getCurrentPosition());
            if (robot.movement != null) {
                telemetry.addData("lf Expected Pos", robot.movement.expectedLf);
                telemetry.addData("rf Expected Pos", robot.movement.expectedRf);
                telemetry.addData("lb Expected Pos", robot.movement.expectedLb);
                telemetry.addData("rb Expected Pos", robot.movement.expectedRb);
                telemetry.addData("movement instance", robot.movement);
                telemetry.addData("movement type", robot.movement.type);
            }
            telemetry.update();
        }
    }
}