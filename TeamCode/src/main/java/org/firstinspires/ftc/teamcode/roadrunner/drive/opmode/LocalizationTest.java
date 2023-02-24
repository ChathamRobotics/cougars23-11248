package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    Encoder parallelEncoder = null;
    Encoder perpendicularEncoder = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("rawParallelCV", parallelEncoder.getCorrectedVelocity());
            telemetry.addData("rawParallelCV", perpendicularEncoder.getCorrectedVelocity());

            telemetry.addData("rawParallelRV", parallelEncoder.getRawVelocity());
            telemetry.addData("rawParallelRV", perpendicularEncoder.getRawVelocity());

            telemetry.addData("rawParallelP", parallelEncoder.getCurrentPosition());
            telemetry.addData("rawParallelP", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
