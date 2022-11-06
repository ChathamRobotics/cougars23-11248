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

@TeleOp(name="Auton Test")
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

        while(opModeIsActive())
        {
            if (gamepad1.cross)
            {
                robot.command(3, 10, "move", this);
            }
            if (gamepad1.triangle)
            {
                robot.command(3, 10, "strafe", this);
            }
            if (gamepad1.square)
            {
                robot.command(10, 10, "turn", this);
            }
            if (gamepad1.circle) {
                robot.command(3, 0, "delay", this);
            }
        }
    }
}