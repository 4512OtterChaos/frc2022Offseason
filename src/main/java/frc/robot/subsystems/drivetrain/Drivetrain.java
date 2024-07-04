package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.drivetrain.DriveConstants.*;


public class Drivetrain extends SubsystemBase {
    private Spark left = new Spark(7);
    private Spark right = new Spark(8);
    private DifferentialDriveWheelSpeeds targetSpeeds = new DifferentialDriveWheelSpeeds();

    public Drivetrain(){
        // left.configFactoryDefault();
        // right.configFactoryDefault();

        right.setInverted(true);

        // left.configOpenloopRamp(kRampRate);
        // right.configOpenloopRamp(kRampRate);

        // left.configVoltageCompSaturation(kVoltageSaturation);
        // left.enableVoltageCompensation(true);
        // left.configContinuousCurrentLimit(kContinuousCurrentLimit);
        // right.configVoltageCompSaturation(kVoltageSaturation);
        // right.enableVoltageCompensation(true);
        // right.configContinuousCurrentLimit(kContinuousCurrentLimit);
    }

    @Override
    public void periodic() {
        double leftVolts = kLinearFF.calculate(targetSpeeds.leftMetersPerSecond);
        double rightVolts = kLinearFF.calculate(targetSpeeds.rightMetersPerSecond);
        right.set(rightVolts / kVoltageSaturation);
        left.set((leftVolts / kVoltageSaturation)*0.862);
    }
    public void stop(){
        right.set(0);
        left.set(0);
    }

    public DifferentialDriveKinematics getKinematics() {
        return kKinematics;
    }

    public double getMaxLinearVelocity() {
        return kLinearFF.maxAchievableVelocity(12, 0);
    }
        
    public void arcadeDrive(double forward, double turn){
        var speeds = DifferentialDrive.arcadeDriveIK(forward, turn, false);
        speeds.left *= getMaxLinearVelocity();
        speeds.right *= getMaxLinearVelocity();
        targetSpeeds = new DifferentialDriveWheelSpeeds(speeds.left, speeds.right);
    }
}
