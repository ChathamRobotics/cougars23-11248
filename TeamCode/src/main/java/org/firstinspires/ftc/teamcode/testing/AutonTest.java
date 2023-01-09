package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonBot;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Op mode for testing the connection and direction of motors
 * Use dpad up and down to adjust power and left and right to select a motor
 * Move the left joystick up and down to run the motor
 * NOTE: This does not respect the motor directions set in OurRobot, all motors are set to run forwards
 */

@TeleOp(name="Auton Test", group="Testing")
public class AutonTest extends LinearOpMode
{
    private final AutonBot robot = new AutonBot();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        robot.init(hardwareMap);

        waitForStart();

        robot.command(5, 75, "strafe", this);

        while(opModeIsActive())
        {
            if (gamepad1.cross)
            {
                robot.command(3, 1, "claw", this);
            }
            if (gamepad1.triangle)
            {
                robot.command(3, 0, "claw", this);
            }
            if (gamepad1.square)
            {
                robot.command(5, 1, "moveClaw", this);
            }
            if (gamepad1.circle) {
                robot.command(5, -90, "turn", this);
            }
        }
    }
}