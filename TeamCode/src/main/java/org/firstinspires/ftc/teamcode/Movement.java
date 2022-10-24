package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Movement {
    public String type;
    public double moveTime;
    public ElapsedTime runtime;
    public double distance;
    public double expectedLf;
    public double expectedLb;
    public double expectedRf;
    public double expectedRb;

    final private double initialLf;
    final private double initialLb;
    final private double initialRf;
    final private double initialRb;

    public Movement(String setType, double setMoveTime, double setDistance, AutonBot robot) {
        type = setType;
        moveTime = setMoveTime;
        distance = setDistance;

        initialLf = robot.leftFront.getCurrentPosition();
        initialLb = robot.leftBack.getCurrentPosition();
        initialRf = robot.rightFront.getCurrentPosition();
        initialRb = robot.rightBack.getCurrentPosition();

        runtime = new ElapsedTime();
    }

    public void frame() {
        switch (type) {
            case "move":
                expectedLf = (distance * runtime.seconds() / moveTime) + initialLf;
                expectedRf = (distance * runtime.seconds() / moveTime) + initialRf;
                expectedLb = (distance * runtime.seconds() / moveTime) + initialLb;
                expectedRb = (distance * runtime.seconds() / moveTime) + initialRb;
                break;
            case "turn":
                expectedLf = (distance * runtime.seconds() / moveTime) + initialLf;
                expectedRf = (distance * runtime.seconds() / moveTime * -1) + initialRf;
                expectedLb = (distance * runtime.seconds() / moveTime) + initialLb;
                expectedRb = (distance * runtime.seconds() / moveTime * -1) + initialRb;
                break;
            case "strafe":
                expectedLf = (distance * runtime.seconds() / moveTime) + initialLf;
                expectedRf = (distance * runtime.seconds() / moveTime * -1) + initialRf;
                expectedLb = (distance * runtime.seconds() / moveTime * -1) + initialLb;
                expectedRb = (distance * runtime.seconds() / moveTime) + initialRb;
                break;
        }
    }
}
