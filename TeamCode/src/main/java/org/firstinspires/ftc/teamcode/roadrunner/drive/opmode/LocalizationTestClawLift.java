package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_LIFT_MAX;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.drive.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTestClawLift extends LinearOpMode {
    Encoder parallelEncoder = null;
    Encoder perpendicularEncoder = null;

    DcMotorEx clawLift = null;
    DcMotorEx clawLift2 = null;
    DcMotorEx reverseSpool = null;


    @Override
    public void runOpMode() throws InterruptedException {
        BaseMecanumDrive drive = new BaseMecanumDrive(hardwareMap);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));

        clawLift            = hardwareMap.get(DcMotorEx.class, "clawLift");
        clawLift2           = hardwareMap.get(DcMotorEx.class, "clawLift2");
        reverseSpool        = hardwareMap.get(DcMotorEx.class, "reverseSpool");


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.4,
                            -gamepad1.left_stick_x * 0.4,
                            -gamepad1.right_stick_x * 0.4
                    )
            );

            drive.update();

            clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            clawLift.setDirection(DcMotorSimple.Direction.REVERSE);
            clawLift2.setDirection(DcMotorSimple.Direction.FORWARD);
            reverseSpool.setDirection(DcMotorSimple.Direction.FORWARD);

            clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            if ((gamepad2.left_stick_y * 0.5 * -1 > 0 && clawLift.getCurrentPosition() < CLAW_LIFT_MAX - 50) || (gamepad2.left_stick_y * 0.5 * -1 < 0 && clawLift.getCurrentPosition() > 100) || (!(clawLift.getCurrentPosition() <= 100) && gamepad2.left_stick_y != 0 && !(clawLift.getCurrentPosition() >= CLAW_LIFT_MAX - 50))) {
                if (gamepad2.left_stick_y * -1 < 0) {
                    clawLift.setPower(0);
                    clawLift2.setPower(0);
                } else if (gamepad2.left_stick_y != 0) {
                    clawLift.setPower(gamepad2.left_stick_y * 0.5 * -1);
                    clawLift2.setPower(gamepad2.left_stick_y * 0.5 * -1);
                    clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    telemetry.addData("currently moving", "clawLift");
                } else {
                    clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (clawLift.getCurrentPosition() >= 100 && clawLift.getCurrentPosition() <= CLAW_LIFT_MAX - 50) {
                clawLift.setPower(0);
                clawLift2.setPower(0);
            } else if(clawLift.getCurrentPosition() < 100) {
                if (clawLift.getCurrentPosition() < 0) {
                    clawLift.setPower(0.5 * 0.1);
                    clawLift2.setPower(0.5 * 0.1);
                } else {
                    clawLift.setPower(0.5 * 0.1 * -1);
                    clawLift2.setPower(0.5 * 0.1 * -1);
                }
                clawLift.setTargetPosition(0);
                clawLift2.setTargetPosition(0);
                clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if(clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 50) {
                if (clawLift.getCurrentPosition() < CLAW_LIFT_MAX) {
                    clawLift.setPower(0.5 * 0.1);
                    clawLift2.setPower(0.5 * 0.1);
                } else {
                    clawLift.setPower(0.5 * 0.1 * -1);
                    clawLift2.setPower(0.5 * 0.1 * -1);
                }
                clawLift.setTargetPosition(CLAW_LIFT_MAX);
                clawLift2.setTargetPosition(CLAW_LIFT_MAX);
                clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.left_stick_y * -1 < 0) {
                reverseSpool.setPower(gamepad2.left_stick_y * 0.5 * -1);
                //clawLift.setPower(gamepad2.left_stick_y * clawLiftPower * REVERSE_SPOOL_MULTI * -1);
                //clawLift2.setPower(gamepad2.left_stick_y * clawLiftPower * REVERSE_SPOOL_MULTI * -1);
                telemetry.addData("currently moving", "reverseSpool");
            } else if (gamepad2.left_stick_y != 0) {
                reverseSpool.setPower(0);
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                reverseSpool.setPower(0);
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

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
