package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_LIFT_MAX;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_BOTTOM;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Movement {
    public String type;
    public double moveTime;
    public double maxPower;
    public ElapsedTime runtime;
    public double distance;
    public double encDistance;

    public double frameCount = 0;

    public double expectedLf;
    public double expectedLb;
    public double expectedRf;
    public double expectedRb;
    public double expectedClawLift;
    public double expectedClaw;
    public double expectedClawIntake;

    public double lfPower;
    public double lbPower;
    public double rfPower;
    public double rbPower;
    public double clawLiftPower;
    public double clawIntakePower;

    /**
    * Class for a movement action <br/>
    * Can be: move, turn, strafe, delay, moveClaw, or claw <br/>
    * - For move and strafe, distance is in inches <br/>
    * - For turn, distance is in degrees <br/>
    * - For delay, distance is not used <br/>
    * - For claw, distance is either 0 for open or 1 for closed <br/>
    * - For moveClaw, distance is from 0 (min) to 1 (max) <br/>
    * - For clawIntake, distance is 0 for ground, 1 for all the way up<br/>
    */
    public Movement(String setType, double setMoveTime, double setDistance, double setMaxPower) {
        type = setType;
        moveTime = setMoveTime;
        distance = setDistance;
        encDistance = setDistance * Constants.ENCODER_COUNTS_PER_IN;
        maxPower = setMaxPower;
    }

    public void init(AutonBot robot) {
        double initialLf = robot.leftFront.getCurrentPosition();
        double initialLb = robot.leftBack.getCurrentPosition();
        double initialRf = robot.rightFront.getCurrentPosition();
        double initialRb = robot.rightBack.getCurrentPosition();

        double initialClawLift = robot.clawLift.getCurrentPosition();
        double initialClaw = robot.clawL.getPosition();
        double initialClawIntake = robot.clawIntake.getCurrentPosition();

        runtime = new ElapsedTime();

        expectedLf = initialLf;
        expectedRf = initialRf;
        expectedLb = initialLb;
        expectedRb = initialRb;

        expectedClawLift = initialClawLift;
        expectedClaw = initialClaw;
        expectedClawIntake = initialClawIntake;
        switch (type) {
            case "move":
                expectedLf = initialLf + encDistance;
                expectedRf = initialRf + encDistance;
                expectedLb = initialLb + encDistance;
                expectedRb = initialRb + encDistance;
                break;
            case "turn":
                expectedLf = initialLf + (encDistance * 0.24);
                expectedRf = initialRf - (encDistance * 0.24);
                expectedLb = initialLb + (encDistance * 0.24);
                expectedRb = initialRb - (encDistance * 0.24);
                break;
            case "strafe":
                expectedLf = initialLf + (encDistance * 12/11);
                expectedRf = initialRf - (encDistance * 12/11);
                expectedLb = initialLb - (encDistance * 12/11);
                expectedRb = initialRb + (encDistance * 12/11);
                break;
            case "delay":
                break;
            case "claw":
                expectedClaw = distance;
                break;
            case "moveClaw":
                expectedClawLift = distance * CLAW_LIFT_MAX;
                break;
            case "clawIntake":
                expectedClawIntake = INTAKE_BOTTOM - distance * INTAKE_BOTTOM;
                break;
        }
    }
    public void frame(AutonBot robot) {
        /*
        lfPower = Math.max(Math.min(Math.pow(expectedLf - robot.leftFront.getCurrentPosition(), 1d/3d) / 25d, maxPower), maxPower * -1);
        rfPower = Math.max(Math.min(Math.pow(expectedRf - robot.rightFront.getCurrentPosition(), 1d/3d) / 25d, maxPower), maxPower * -1);
        lbPower = Math.max(Math.min(Math.pow(expectedLb - robot.leftBack.getCurrentPosition(), 1d/3d) / 25d, maxPower), maxPower * -1);
        rbPower = Math.max(Math.min(Math.pow(expectedRb - robot.rightBack.getCurrentPosition(), 1d/3d) / 25d, maxPower), maxPower * -1);
         */

        lfPower = Math.max(Math.min(expectedLf - robot.leftFront.getCurrentPosition() / 25d / 25d, maxPower), maxPower * -1);
        rfPower = Math.max(Math.min(expectedRf - robot.rightFront.getCurrentPosition() / 25d / 25d, maxPower), maxPower * -1);
        lbPower = Math.max(Math.min(expectedLb - robot.leftBack.getCurrentPosition() / 25d / 25d, maxPower), maxPower * -1);
        rbPower = Math.max(Math.min(expectedRb - robot.rightBack.getCurrentPosition() / 25d / 25d, maxPower), maxPower * -1);


        clawLiftPower = Math.abs(Math.max(Math.min(expectedClawLift - robot.clawLift.getCurrentPosition() / 25d / 25d, maxPower), maxPower * -1));
        clawIntakePower = Math.max(Math.min(expectedClawIntake - robot.clawIntake.getCurrentPosition() / 25d / 25d, maxPower), maxPower * -1);
        frameCount += 1;
    }
}
