package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonBot;
import org.firstinspires.ftc.teamcode.Constants;

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
