package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.TeleopBot;

@TeleOp(name="Ultrasonic Test", group="Testing")
public class UltrasonicTest extends LinearOpMode
{
    private final TeleopBot robot = new TeleopBot();
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastPowerChangeTime;
    private double clawLiftPower = 0.5d;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Robot Ready");
        telemetry.update();

        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Status", "Running");
            telemetry.addData("Ultrasonic - RU", robot.ultrasonic.rawUltrasonic());
            telemetry.addData("Ultrasonic - CMU", robot.ultrasonic.cmUltrasonic());
            telemetry.addData("Ultrasonic - RO", robot.ultrasonic.rawOptical());
            telemetry.addData("Ultrasonic - CMO", robot.ultrasonic.cmOptical());
            telemetry.update();
        }
    }
}