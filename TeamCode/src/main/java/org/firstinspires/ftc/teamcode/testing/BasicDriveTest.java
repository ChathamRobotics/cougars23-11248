package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Op mode for testing the connection and direction of motors
 * Use dpad up and down to adjust power and left and right to select a motor
 * Move the left joystick up and down to run the motor
 * NOTE: This does not respect the motor directions set in OurRobot, all motors are set to run forwards
 */
@TeleOp(name="Basic Drive Test", group="Testing")
public class BasicDriveTest extends LinearOpMode
{
    private double power = 0.5;
    private double lastPowerChangeTime;
    private final ElapsedTime runtime = new ElapsedTime();
    private int motorIndex = 0;
    private boolean leftDownLastFrame = false;
    private boolean rightDownLastFrame = false;

    @Override
    public void runOpMode()
    {
        // Get all the motors and do basic setup on each one
        DcMotor lf = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor lb = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rightBack");

        lf.setDirection(Direction.FORWARD);
        rf.setDirection(Direction.FORWARD);
        lb.setDirection(Direction.FORWARD);
        rb.setDirection(Direction.REVERSE);

        lf.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(RunMode.RUN_USING_ENCODER);
        rf.setMode(RunMode.RUN_USING_ENCODER);
        lb.setMode(RunMode.RUN_USING_ENCODER);
        rb.setMode(RunMode.RUN_USING_ENCODER);

        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            // Adjust Power
            if (runtime.time() > lastPowerChangeTime + 0.5)
            {
                if (gamepad1.dpad_down)
                {
                    power = Math.max(0, power - 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad1.dpad_up)
                {
                    power = Math.min(1, power + 0.1);
                    lastPowerChangeTime = runtime.time();
                }
            }


            // Actually turn the selected motor
            lf.setPower(gamepad1.left_stick_y * -1 * power);
            lb.setPower(gamepad1.left_stick_y * -1 * power);
            rf.setPower(gamepad1.left_stick_y * -1 * power);
            rb.setPower(gamepad1.left_stick_y * -1 * power);

            // Display the current motor name, encoder position, and power
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}