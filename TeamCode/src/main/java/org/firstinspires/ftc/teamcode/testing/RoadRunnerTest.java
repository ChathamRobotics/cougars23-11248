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

        TrajectorySequence setup = drive.trajectorySequenceBuilder(new Pose2d(-33.1, -62.58, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-35.20, -62.79))
                .lineToConstantHeading(new Vector2d(-37.13, -11.79))
                .lineToLinearHeading(new Pose2d(-44.79, -15.82, Math.toRadians(-1)))
                .build();

        TrajectorySequence toJunction = drive.trajectorySequenceBuilder(setup.end())
                .lineToLinearHeading(new Pose2d(-38.62, -12.24, Math.toRadians(36.67)))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.setClawIntakePosition(0, this);
        drive.setClawIntakePosition(0.2, this);
        drive.followTrajectorySequence(setup);
        drive.followTrajectorySequence(toJunction);
        drive.setClawIntakePosition(0, this);
        drive.setClawLiftPosition(1, this);
        drive.setClawPosition(0, true);
        drive.setClawIntakePosition(0.6, this);
        drive.setClawLiftPosition(0, this);
        drive.setClawIntakePosition(0, this);
        while (true) {

        }
    }

}
