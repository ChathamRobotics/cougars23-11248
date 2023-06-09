package org.firstinspires.ftc.teamcode.archive.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.archive.auton.AutonBot;

@Disabled
@Autonomous(name = "Basic Auton", group = "Basic")
public class BasicAuton extends LinearOpMode {
    AutonBot robot = new AutonBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);

        waitForStart();

        robot.command(3, -15, "strafe");
        robot.command(3, -15, "strafe");
        robot.command(3, 24, "move");
    }
}
