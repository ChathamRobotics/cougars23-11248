package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonBot;
import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name = "Basic Auton")
public class BasicAuton extends LinearOpMode {
    AutonBot robot = new AutonBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        //robot.command(3, 10, "strafe");
        //robot.command(3, 12.5, "turn");
        //robot.command(3, 10, "move");
        //robot.command(3, 3, "turn");
        //robot.command(3, 3, "move");

        robot.command(10, 20, "move", this);
        //robot.command(2, 30, "turn");

    }
}
