package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_LIFT_MAX;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_BOTTOM;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;


@TeleOp(name = "Adv Roadrunner Drive")
public class AdvRoadrunnerDrive extends LinearOpMode {
    Encoder parallelEncoder = null;
    Encoder perpendicularEncoder = null;

    DcMotorEx clawLift = null;
    DcMotorEx clawLift2 = null;
    DcMotorEx reverseSpool = null;
    DcMotor clawIntake = null;

    Servo clawL = null;
    Servo clawR = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private double lastPowerChangeTime;
    private boolean boost = false;
    private double basePower = 0.4;
    private double clawLiftPower = 1;
    private double clawIntakePower = 0.3;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));

        clawLift            = hardwareMap.get(DcMotorEx.class, "clawLift");
        clawLift2           = hardwareMap.get(DcMotorEx.class, "clawLift2");
        clawIntake          = hardwareMap.get(DcMotorEx.class, "clawIntake");
        reverseSpool        = hardwareMap.get(DcMotorEx.class, "reverseSpool");
        clawL               = hardwareMap.get(Servo.class, "clawL");
        clawR               = hardwareMap.get(Servo.class, "clawR");

        clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        clawLift.setDirection(DcMotorSimple.Direction.REVERSE);
        clawLift2.setDirection(DcMotorSimple.Direction.FORWARD);
        reverseSpool.setDirection(DcMotorSimple.Direction.FORWARD);
        clawIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        reverseSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawLift.setPower(0);
        clawLift2.setPower(0);
        reverseSpool.setPower(0);
        clawIntake.setPower(0);

        clawL.scaleRange(0.07, 0.21);
        clawR.scaleRange(0.57, 0.71);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            // Fine Adjust Power
            if (runtime.time()  > lastPowerChangeTime + 0.5)
            {
                if (gamepad1.dpad_down)
                {
                    basePower = Math.max(0, basePower - 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad1.dpad_up)
                {
                    basePower = Math.min(1, basePower + 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                if (gamepad2.dpad_down)
                {
                    clawLiftPower = Math.max(0, clawLiftPower - 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad2.dpad_up)
                {
                    clawLiftPower = Math.min(1, clawLiftPower + 0.1);
                    lastPowerChangeTime = runtime.time();
                }
            }

            // Quick Adjust Power
            if (gamepad1.left_bumper) {
                basePower = 0.9;
                boost = true;
            } else if (gamepad1.right_bumper) {
                basePower = 0.2;
                boost = true;
            } else if (!gamepad1.right_bumper && !gamepad1.left_bumper && boost) {
                basePower = 0.4;
                boost = false;
            }


            // Claw
            if (gamepad2.left_bumper) {
                // open
                clawL.setPosition(1);
                clawR.setPosition(0);
            }
            else if (gamepad2.right_bumper) {
                // close
                clawL.setPosition(0);
                clawR.setPosition(1);
            }
            else if (gamepad2.right_trigger > 0) {
                // slightly more closed than halfway
                clawL.setPosition(0.25);
                clawR.setPosition(0.75);
            }

            // Clawlift Slowdown & Brake
            if (gamepad2.left_trigger > 0) {
                clawIntakePower = 0.075;
            } else {
                clawIntakePower = 0.3;
            }

            if (gamepad2.cross) {
                clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // ClawLift reset encoders
            if (gamepad2.triangle) { // AKA Y
                clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                clawLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                reverseSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.circle) { // AKA B
                clawIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * basePower,
                            -(gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x) * basePower * 1.2,
                            -gamepad1.right_stick_x * basePower
                    )
            );
            drive.update();




            if ((gamepad2.left_stick_y * clawLiftPower * -1 > 0 && clawLift.getCurrentPosition() < CLAW_LIFT_MAX - 50) || (gamepad2.left_stick_y * clawLiftPower * -1 < 0 && clawLift.getCurrentPosition() > 100) || (!(clawLift.getCurrentPosition() <= 100) && gamepad2.left_stick_y != 0 && !(clawLift.getCurrentPosition() >= CLAW_LIFT_MAX - 50))) {
                if (gamepad2.left_stick_y * -1 < 0) {
                    clawLift.setPower(0);
                    clawLift2.setPower(0);
                } else if (gamepad2.left_stick_y != 0) {
                    clawLift.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
                    clawLift2.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
                    clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    telemetry.addData("currently moving", "clawLift");
                    telemetry.addData("clawLift Power", clawLift.getPower());
                    telemetry.addData("clawLift2 Power", clawLift2.getPower());
                } else {
                    clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (clawLift.getCurrentPosition() >= 100 && clawLift.getCurrentPosition() <= CLAW_LIFT_MAX - 50) {
                clawLift.setPower(0);
                clawLift2.setPower(0);
            } else if(clawLift.getCurrentPosition() < 100) {
                if (clawLift.getCurrentPosition() < 0) {
                    clawLift.setPower(clawLiftPower * 0.1);
                    clawLift2.setPower(clawLiftPower * 0.1);
                } else {
                    clawLift.setPower(clawLiftPower * 0.1 * -1);
                    clawLift2.setPower(clawLiftPower * 0.1 * -1);
                }
                clawLift.setTargetPosition(0);
                clawLift2.setTargetPosition(0);
                clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if(clawLift.getCurrentPosition() > CLAW_LIFT_MAX - 50) {
                if (clawLift.getCurrentPosition() < CLAW_LIFT_MAX) {
                    clawLift.setPower(clawLiftPower * 0.1);
                    clawLift2.setPower(clawLiftPower * 0.1);
                } else {
                    clawLift.setPower(clawLiftPower * 0.1 * -1);
                    clawLift2.setPower(clawLiftPower * 0.1 * -1);
                }
                clawLift.setTargetPosition(CLAW_LIFT_MAX);
                clawLift2.setTargetPosition(CLAW_LIFT_MAX);
                clawLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.left_stick_y * -1 < 0) {
                reverseSpool.setPower(gamepad2.left_stick_y * clawLiftPower * -1);
                telemetry.addData("currently moving", "reverseSpool");
            } else if (gamepad2.left_stick_y != 0) {
                reverseSpool.setPower(0);
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                telemetry.addData("reverse spool", "FLOAT");
            } else {
                reverseSpool.setPower(0);
                reverseSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if ((gamepad2.right_stick_y * -1 < 0 && clawIntake.getCurrentPosition() > INTAKE_BOTTOM) || (gamepad2.right_stick_y * -1 > 0 && clawIntake.getCurrentPosition() < 0)) {
                clawIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (Math.abs(gamepad2.right_stick_y) > 0.99) {
                    clawIntake.setPower(gamepad2.right_stick_y * clawIntakePower * - 1);
                }
                else {
                    clawIntake.setPower(gamepad2.right_stick_y * clawIntakePower * - 1 * 0.2);
                }
            } else if (clawIntake.getCurrentPosition() > 0) {
                clawIntake.setPower(-0.05);
                clawIntake.setTargetPosition(0);
                clawIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                clawIntake.setPower(0);
            }

            //Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Power", basePower);
            telemetry.addData("Claw Lift Power", clawLiftPower);
            telemetry.addData("ClawL Servo Position", clawL.getPosition());
            telemetry.addData("ClawR Servo Position", clawR.getPosition());
            telemetry.addData("Claw Lift Position", clawLift.getCurrentPosition());
            telemetry.addData("Claw Lift Power", clawLift.getPower());
            telemetry.addData("Reverse Spool Position", reverseSpool.getCurrentPosition());
            telemetry.addData("Reverse Spool Target Position", reverseSpool.getTargetPosition());
            telemetry.addData("Reverse Spool Power", reverseSpool.getPower());
            telemetry.addData("A", gamepad2.left_stick_y);
            telemetry.addData("Claw Intake Pos", clawIntake.getCurrentPosition());
            telemetry.addData("Claw Intake Power", clawIntakePower);
            telemetry.addData("Claw Intake Speed", clawIntake.getPower());
            telemetry.update();

            // Fix reverse spool
            if (gamepad1.triangle) {
                reverseSpool.setPower(1);
                reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.square) {
                reverseSpool.setPower(-1);
                reverseSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                clawLift.setPower(0);
                clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                clawLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                clawLift2.setPower(0);
                clawLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                clawLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
}
