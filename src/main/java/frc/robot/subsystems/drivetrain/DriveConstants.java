package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class DriveConstants{

    public static final double kWheelDiameter = Units.inchesToMeters(6);

    public static final double kRampRate = 0.08;

    public static final DifferentialDriveKinematics kKinematics =
            new DifferentialDriveKinematics(Units.inchesToMeters(24.25));
    // Linear drive feedforward (forward/backwards)
    public static final SimpleMotorFeedforward kLinearFF = new SimpleMotorFeedforward(
        1, // Voltage to break static friction
        3, // Volts per meter per second
        0.3 // Volts per meter per second squared
    );
    
    // Wheel velocity PID
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kVoltageSaturation = 12;
    // public static final int kContinuousCurrentLimit = 40;
    // public static final int kPeakCurrentLimit = 60;
}