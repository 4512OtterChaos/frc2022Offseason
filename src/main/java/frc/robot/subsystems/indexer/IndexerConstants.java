package frc.robot.subsystems.indexer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IndexerConstants {
    public static final int kMotorID = 10;
    public static final int kBottomSensorID = 1;
    public static final int kTopSensorID = 0;
    
    public static final boolean kMotorInverted = true;

    public static final double kVoltageIn = 3; // intake/index
    public static final double kVoltageFeed = 5; // feed shooter
    public static final double kVoltageOut = -5;

    // Feedforward loaded with cargo (simulation rn)
    public static final SimpleMotorFeedforward kFF = new SimpleMotorFeedforward(
        0.28, // Voltage to break static friction
        0.02, // Volts per radian per second
        0.005 // Volts per radian per second squared
    );
    
    // Current limits
    public static final int kContinuousCurrentLimit = 40;
    public static final int kPeakCurrentLimit = 50;
    public static final double kPeakCurrentDuration = 0.1;
    // Voltage compensation`
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;
    public static final int kCANTimeout = 100;

    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
}
