package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.auton.AutonConstants.*;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Left Full Auton", group="1")

public class LeftFullAuton extends LinearOpMode {

    OpenCvWebcam webcam;
    ElapsedTime runtime = new ElapsedTime();
    MecanumDrive drive = null;

    @Override
    public void runOpMode() {
        //
        //     Setup Camera
        //
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        ColorDetectingPipeline pipeline = new ColorDetectingPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("error", errorCode);
            }
        });




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

        if(isStopRequested()) return;


        //
        //     Detect Color
        //

        double totalPurplePixels = 0;
        double totalGreenPixels = 0;
        double totalOrangePixels = 0;
        double cycles = 0;

        while (runtime.seconds() < 0.5) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", "%.2f", webcam.getFps());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Pixel colors (PGO)", "[" + pipeline.getPixelColors()[0] + ", " + pipeline.getPixelColors()[1] + ", " + pipeline.getPixelColors()[2] + "]");
            telemetry.update();

            if (pipeline.getPixelColors()[0] + pipeline.getPixelColors()[1] + pipeline.getPixelColors()[2] != 0) {
                totalPurplePixels += pipeline.getPixelColors()[0];
                totalGreenPixels += pipeline.getPixelColors()[1];
                totalOrangePixels += pipeline.getPixelColors()[2];

                cycles += 1;
                Log.d("Color Detection", "(PGO:) [" + totalPurplePixels + ", " + totalGreenPixels + ", " + totalOrangePixels + "] Cycles: " + cycles);
            }
        }
        webcam.stopStreaming();

        double[] avgPixelColors = {totalPurplePixels / cycles, totalGreenPixels / cycles, totalOrangePixels / cycles};
        // Offset for env pixels
        avgPixelColors[0] -= 0; // Purple
        avgPixelColors[1] -= 0; // Green
        avgPixelColors[2] -= 0; // Orange

        int maxPixels = 0;
        int nextPixels = 0;
        for (int i = 0; i < avgPixelColors.length; i++) {
            if (avgPixelColors[i] > avgPixelColors[maxPixels]) {
                nextPixels = maxPixels;
                maxPixels = i;
            }
        }

        String[] colorNames = {"purple", "green", "orange"};

        telemetry.addData("Max", (int)(avgPixelColors[maxPixels] * 100) / 100 + ": " + colorNames[maxPixels]);
        telemetry.addData("Next", (int)(avgPixelColors[nextPixels] * 100) / 100 + ": " + colorNames[nextPixels]);
        telemetry.addData("Confidence", (int)((avgPixelColors[maxPixels] - avgPixelColors[nextPixels]) / avgPixelColors[maxPixels] * 100 * 100) / 100 + "%");
        telemetry.update();


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


        //
        //     Park
        //

        switch (maxPixels) {
            case 0:
                // purple - zone 1
                drive.followTrajectorySequence(parkZone1);
                break;
            case 1:
                // green - zone 2
                drive.followTrajectorySequence(parkZone2);
                break;
            case 2:
                // orange - zone 3
                drive.followTrajectorySequence(parkZone3);
                break;
        }

        while (opModeIsActive()) {
            sleep(10);
        }
    }



}