package org.firstinspires.ftc.teamcode;

public class Constants {
    /*
    * Constants for encoder-based drive
    * https://www.revrobotics.com/rev-45-1655/
    * https://www.revrobotics.com/rev-41-1600/
    */

    // Base # of the enc number per revolution of the motor
    private static final Integer BaseEncoderCountsPerRev = 28;
    // # of enc counts the motor needs after going through various gear ratios (if ratio is x:1, multiply by x/1)
    private static final Double EncoderCountsPerRev = BaseEncoderCountsPerRev * 15.15;
    // In cm
    private static final Double WheelDiameter = 7.5;
    // Get the final enc counts per cm by getting the amount per rev and dividing by the wheel circumference (amount moved per rev)
    public static final Double ENCODER_COUNTS_PER_CM = EncoderCountsPerRev / WheelDiameter / Math.PI;
    public static final Double ENCODER_COUNTS_PER_IN = ENCODER_COUNTS_PER_CM * 2.54;
}
