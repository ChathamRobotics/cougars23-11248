package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name="Road Runner Test", group="Testing")

public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d redLeftStart = new Pose2d(-62.58, 33.82, 0);

        drive.setPoseEstimate(redLeftStart);

        Trajectory myTrajectory = drive.trajectoryBuilder(redLeftStart)
                .splineToConstantHeading(new Vector2d(-12d, 33.82), 0)
                .splineToConstantHeading(new Vector2d(-12d, 0), 0)
                .splineToConstantHeading(new Vector2d(), 0)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }

}
