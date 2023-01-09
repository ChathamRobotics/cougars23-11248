package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;

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

    public double lfPower;
    public double lbPower;
    public double rfPower;
    public double rbPower;
    public double clawLiftPower;

    /**
    * Class for a movement action <br/>
    * Can be: move, turn, strafe, delay, moveClaw, or claw <br/>
    * - For move and strafe, distance is in inches <br/>
    * - For turn, distance is in degrees <br/>
    * - For delay, distance is not used <br/>
    * - For claw, distance is either 1 for open or 0 for closed <br/>
    * - For moveClaw, distance is 0 for ground, 1 for max height, -1 for short junction, -2 for medium junction, and -3 for tall junction <br/>
     */
    public Movement(String setType, double setMoveTime, double setDistance, double setMaxPower, AutonBot robot) {
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

        runtime = new ElapsedTime();

        switch (type) {
            case "move":
                expectedLf = initialLf + encDistance;
                expectedRf = initialRf + encDistance;
                expectedLb = initialLb + encDistance;
                expectedRb = initialRb + encDistance;

                expectedClawLift = initialClawLift;
                expectedClaw = initialClaw;
                break;
            case "turn":
                expectedLf = initialLf + (encDistance * 12.5 / 90 * 1.5);
                expectedRf = initialRf - (encDistance * 12.5 / 90 * 1.5);
                expectedLb = initialLb + (encDistance * 12.5 / 90 * 1.5);
                expectedRb = initialRb - (encDistance * 12.5 / 90 * 1.5);

                expectedClawLift = initialClawLift;
                expectedClaw = initialClaw;
                break;
            case "strafe":
                Log.d("Movement", initialLf - (encDistance * 2) + "");
                Log.d("Movement", initialRf - (encDistance * 2) + "");
                Log.d("Movement", initialLb - (encDistance * 2) + "");
                Log.d("Movement", initialRb - (encDistance * 2) + "");

                expectedLf = initialLf + (encDistance * 2);
                expectedRf = initialRf - (encDistance * 2);
                expectedLb = initialLb - (encDistance * 2);
                expectedRb = initialRb + (encDistance * 2);

                expectedClawLift = initialClawLift;
                expectedClaw = initialClaw;
                break;
            case "delay":
                expectedLf = initialLf;
                expectedRf = initialRf;
                expectedLb = initialLb;
                expectedRb = initialRb;

                expectedClawLift = initialClawLift;
                expectedClaw = initialClaw;
                break;
            case "claw":
                expectedLf = initialLf;
                expectedRf = initialRf;
                expectedLb = initialLb;
                expectedRb = initialRb;

                expectedClawLift = initialClawLift;
                expectedClaw = distance;
                break;
            case "moveClaw":
                expectedLf = initialLf;
                expectedRf = initialRf;
                expectedLb = initialLb;
                expectedRb = initialRb;

                switch ((int)distance) {
                    case -3:
                        expectedClawLift = 1800;
                        break;
                    case -2:
                        expectedClawLift = 1000;
                        break;
                    case -1:
                        expectedClawLift = 500;
                        break;
                    case 0:
                    case 1:
                        expectedClawLift = distance * 1800;
                        break;
                    default:
                        expectedClawLift = initialClawLift;


                }
                expectedClawLift = distance * 1800;
                expectedClaw = initialClaw;
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


        clawLiftPower = Math.max(Math.min(Math.pow(expectedRb - robot.clawLift.getCurrentPosition(), 1d/3d) / 25d, maxPower), maxPower * -1);
        frameCount += 1;
    }
}
