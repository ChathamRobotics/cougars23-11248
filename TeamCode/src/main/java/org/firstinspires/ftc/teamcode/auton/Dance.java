package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.auton.AutonConstants.leftJunction;
import static org.firstinspires.ftc.teamcode.auton.AutonConstants.leftStacks;
import static org.firstinspires.ftc.teamcode.auton.AutonConstants.leftZone1;
import static org.firstinspires.ftc.teamcode.auton.AutonConstants.leftZone2;
import static org.firstinspires.ftc.teamcode.auton.AutonConstants.leftZone3;
import static org.firstinspires.ftc.teamcode.auton.AutonConstants.redLeftStart;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="Left Just Cones", group="1")

public class Dance extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    MecanumDrive drive = null;

    @Override
    public void runOpMode() {
        //
        //     Setup Camera
        //
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        //
        //     Setup RoadRunner + Trajectories
        //

        drive = new MecanumDrive(hardwareMap);

        drive.setClawPosition(1, false);

        drive.setPoseEstimate(redLeftStart);


        TrajectorySequence setup = drive.trajectorySequenceBuilder(redLeftStart)
                .lineToConstantHeading(new Vector2d(-35.20, -62.79))
                .lineToConstantHeading(new Vector2d(-36.13, -5.87))
                .lineToConstantHeading(new Vector2d(-36.13, -11.87))
                .lineToLinearHeading(leftJunction)
                .build();

        TrajectorySequence toStacks = drive.trajectorySequenceBuilder(leftJunction)
                .splineToLinearHeading(leftStacks, Math.toRadians(20))
                .forward(3)
                .build();

        TrajectorySequence toJunction = drive.trajectorySequenceBuilder(leftStacks)
                .splineToLinearHeading(leftJunction, Math.toRadians(20))
                .build();


        TrajectorySequence parkZone1 = drive.trajectorySequenceBuilder(leftJunction)
                .lineToLinearHeading(leftZone2)
                .lineToLinearHeading(leftZone1)
                .strafeRight(3)
                .build();

        TrajectorySequence parkZone2 = drive.trajectorySequenceBuilder(leftJunction)
                .lineToLinearHeading(leftZone2)
                .strafeRight(3)
                .build();

        TrajectorySequence parkZone3 = drive.trajectorySequenceBuilder(leftJunction)
                .lineToLinearHeading(leftZone2)
                .lineToLinearHeading(leftZone3)
                .strafeRight(3)
                .build();


        //
        //     After start button is pressed
        //

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;


        //
        //     Score preload & stacks
        //

        drive.setClawIntakePosition(0, this);
        drive.followTrajectorySequence(setup);
        drive.scoreConeOnTallJunction(this);
        drive.followTrajectorySequence(toStacks);
        drive.getConeFromStacks(5, this);
        drive.followTrajectorySequence(toJunction);
        drive.scoreConeOnTallJunction(this);

        drive.followTrajectorySequence(parkZone2);


    }
}