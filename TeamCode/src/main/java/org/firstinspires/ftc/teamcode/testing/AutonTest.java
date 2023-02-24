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
    private double multi = 1;
    private double lastMultiChange = 0;

    @Override
    public void runOpMode()
    {
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        robot.init(hardwareMap, this);

        waitForStart();

        while(opModeIsActive())
        {
            if (gamepad1.cross) {
                robot.command(3, 1, "moveClaw");
            }
            if (gamepad1.square) {
                robot.command(3, 0, "moveClaw");
            }
            if (gamepad1.circle) {
                robot.command(3, 0, "clawIntake");
            }
            if (gamepad1.triangle) {
                robot.command(3, 1, "clawIntake");
            }
            if (gamepad1.left_bumper) {
                robot.command(1, 1, "claw");
            }
            if (gamepad1.right_bumper) {
                robot.command(1, 0, "claw");
            }
        }
    }
}