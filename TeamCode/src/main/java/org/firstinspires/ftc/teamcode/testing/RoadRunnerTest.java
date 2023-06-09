package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="Road Runner Test", group="Testing")

public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        drive.setClawPosition(1, false);

        Pose2d redLeftStart = new Pose2d(-33.1, -62.58, Math.toRadians(90.00)); // From perspective of side, Red L, Blue R

        drive.setPoseEstimate(redLeftStart);

        Pose2d junction = new Pose2d(-35.69, -13.26, Math.toRadians(39.28));
        Pose2d stacks = new Pose2d(-43.03, -15.97, Math.toRadians(0));
        Pose2d zone1 = new Pose2d(-56.51, -15.84, Math.toRadians(0));
        Pose2d zone2 = new Pose2d(-33.00, -16.00, Math.toRadians(0));
        Pose2d zone3 = new Pose2d(-8.7, -15.53, Math.toRadians(0));

        TrajectorySequence setup = drive.trajectorySequenceBuilder(new Pose2d(-33.1, -62.58, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-35.20, -62.79))
                .lineToConstantHeading(new Vector2d(-36.13, -5.87))
                .lineToConstantHeading(new Vector2d(-36.13, -13.87))
                .lineToLinearHeading(junction)
                .build();

        TrajectorySequence toStacks = drive.trajectorySequenceBuilder(junction)
                .lineToLinearHeading(stacks)
                .forward(3)
                .build();

        TrajectorySequence toJunction = drive.trajectorySequenceBuilder(stacks)
                .lineToLinearHeading(junction)
                .build();


        TrajectorySequence parkZone1 = drive.trajectorySequenceBuilder(junction)
                .lineToLinearHeading(zone1)
                .build();

        TrajectorySequence parkZone2 = drive.trajectorySequenceBuilder(junction)
                .lineToLinearHeading(zone2)
                .build();

        TrajectorySequence parkZone3 = drive.trajectorySequenceBuilder(junction)
                .lineToLinearHeading(new Pose2d(junction.getX(), junction.getY(), zone3.getHeading()))
                .lineToLinearHeading(zone3)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.setClawIntakePosition(0, this);
        drive.followTrajectorySequence(setup);
        drive.scoreConeOnTallJunction(this);
        drive.followTrajectorySequence(toStacks);
        drive.getConeFromStacks(5, this);
        drive.followTrajectorySequence(toJunction);
        drive.scoreConeOnTallJunction(this);



        while (opModeIsActive()) {
            sleep(10);
        }
    }



}