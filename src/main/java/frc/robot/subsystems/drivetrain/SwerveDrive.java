package frc.robot.subsystems.drivetrain;

import static frc.robot.auto.AutoConstants.*;

import java.util.Optional;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.AutoConstants;
// import frc.robot.subsystems.drivetrain.SwerveModule.SwerveModulesLog;
// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.*;

// @Log.Exclude
public class SwerveDrive extends SubsystemBase{ //implements Loggable {

    // construct our modules in order with their specific constants
    private final SwerveModule[] swerveMods = {
        new SwerveModule(SwerveConstants.Module.FL),
        new SwerveModule(SwerveConstants.Module.FR),
        new SwerveModule(SwerveConstants.Module.BL),
        new SwerveModule(SwerveConstants.Module.BR)
    };

    // auto-config
    // private final SwerveModulesLog logModules = new SwerveModulesLog(swerveMods);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        swerveMods[0].getModuleConstants().centerOffset,
        swerveMods[1].getModuleConstants().centerOffset,
        swerveMods[2].getModuleConstants().centerOffset,
        swerveMods[3].getModuleConstants().centerOffset
    );

    private final WPI_Pigeon2 gyro = new WPI_Pigeon2(SwerveConstants.kPigeonID);
    
    private final SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
    private boolean isFieldRelative = true;

    // path controller and its dimension-specific controllers
    // i.e 1 meter error in the x direction = kP meters per second x velocity added
    // @Config.PIDController
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    // @Config.PIDController
    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    // our auto rotation targets are profiled to obey velocity and acceleration constraints
    // @Config.PIDController
    // private final ProfiledPIDController thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController,
    //     AutoConstants.kThetaControllerConstraints
    // );

    private Trajectory logTrajectory;
    
    public SwerveDrive() {
        
        gyro.configAllSettings(SwerveConstants.kPigeon2Config);
        
        zeroGyro();
        
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // thetaController.setTolerance(kThetaPositionTolerance, kThetaVelocityTolerance);

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            calculateVisionNoise()
        );
    }

    @Override
    public void periodic() {
        for(SwerveModule module : swerveMods) {
            module.periodic();
        }

        // display our robot (and individual modules) pose on the field
        poseEstimator.update(getGyroYaw(), getModulePositions());
    }

    /**
     * Basic teleop drive control; ChassisSpeeds values representing vx, vy, and omega
     * are converted to individual module states for the robot to follow
     * @param vxMeters x velocity (forward)
     * @param vyMeters y velocity (strafe)
     * @param omegaRadians angular velocity (rotation CCW+)
     * @param openLoop If swerve modules should not use velocity PID
     */
    public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean openLoop){
        double vx = vxMeters;
        double vy = vyMeters;
        double omega = omegaRadians;
        ChassisSpeeds targetChassisSpeeds;
        if(isFieldRelative){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading());
        }
        else{
            targetChassisSpeeds = new ChassisSpeeds(vx,vy,omega);
        }
        setChassisSpeeds(targetChassisSpeeds, openLoop, false);
    }
    /**
     * Drive control using angle position (theta) instead of velocity (omega).
     * The {@link #thetaController theta PID controller} calculates an angular velocity in order
     * to reach the target angle, making this method similar to autonomous path following without
     * x/y position controllers. This method assumes field-oriented control and is not affected
     * by the value of {@link #isFieldRelative}.
     * @param vxMeters x velocity (forward)
     * @param vyMeters y velocity (strafe)
     * @param targetRotation target angular position
     * @param openLoop If swerve modules should not use velocity PID
     * @return If the drivetrain rotation is within tolerance of the target rotation
     */
    // public boolean drive(double vxMeters, double vyMeters, Rotation2d targetRotation, boolean openLoop){
    //     // rotation speed
    //     double rotationRadians = getPose().getRotation().getRadians();
    //     double thetaFeedback = thetaController.calculate(rotationRadians, targetRotation.getRadians());
    //     double thetaFF = thetaController.getSetpoint().velocity;

    //     // + translation speed
    //     ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //         vxMeters,
    //         vyMeters,
    //         thetaFF + thetaFeedback,
    //         getHeading()
    //     );

    //     setChassisSpeeds(targetChassisSpeeds, openLoop, false);
    //     return thetaController.atGoal();
    // }

    /**
     * Command the swerve modules to the desired states.
     * Velocites above maximum speed will be downscaled (preserving ratios between modules)
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxLinearSpeed);
        for(int i=0;i<4;i++){
            swerveMods[i].setDesiredState(desiredStates[i], openLoop, steerInPlace);
        }
    }
    /**
     * Uses kinematics to convert ChassisSpeeds to module states.
     * @see {@link #setModuleStates(SwerveModuleState[], boolean)}
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace){
        setModuleStates(kinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
        this.targetChassisSpeeds = targetChassisSpeeds;
    }
    public void stop(){
        drive(0, 0, 0, true);
    }

    /**
     * Changes whether drive methods use field or robot-oriented control.
     */
    public void setIsFieldRelative(boolean is) {isFieldRelative = is;}

    public void setBrakeOn(boolean is){
        for(SwerveModule module : swerveMods) {
            module.setDriveBrake(is);
            module.setSteerBrake(is);
        }
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    public void addVisionMeasurement(Pose2d measurement, double latencySeconds){
        poseEstimator.addVisionMeasurement(
            measurement,
            Timer.getFPGATimestamp() - latencySeconds,
            calculateVisionNoise()
        );
    }
    public void resetOdometry(Pose2d pose){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
    public void resetPathController(){
        xController.reset();
        yController.reset();
        // thetaController.reset(getHeading().getRadians(), getChassisSpeeds().omegaRadiansPerSecond);
    }

    public boolean getIsFieldRelative() {return isFieldRelative;}

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    public Optional<Pose2d> getPose(double secondsAgo) {
        return poseEstimator.getEstimatedPosition(Timer.getFPGATimestamp() - secondsAgo);
    }
    /**
     * Swerve drive rotation on the field reported by odometry.
     */
    public Rotation2d getHeading(){
        return getPose().getRotation();
    }
    /**
     * Raw gyro yaw (this may not match the field heading!).
     */
    public Rotation2d getGyroYaw(){
        return gyro.getRotation2d();
    }
    /**
     * Adjust the measurement noise/trust of vision estimation as robot velocities change.
     */
    private Vector<N3> calculateVisionNoise(){
        ChassisSpeeds speeds = getChassisSpeeds();
        double linearPercent = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / (
            SwerveConstants.kMaxLinearSpeed);
        double angularPercent = Math.abs(speeds.omegaRadiansPerSecond) / SwerveConstants.kMaxAngularSpeed;
        return VecBuilder.fill(
            MathUtil.interpolate(0.05, 10, linearPercent),
            MathUtil.interpolate(0.05, 10, linearPercent),
            Units.degreesToRadians(MathUtil.interpolate(1, 45, angularPercent))
        );
    }

    public double getLinearVelocity(){
        return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
    }
    public double getMaxLinearVelocityMeters(){
        return SwerveConstants.kMaxLinearSpeed;
    }
    public double getMaxAngularVelocityRadians(){
        return SwerveConstants.kMaxAngularSpeed;
    }
    
    /**
     * @return An ordered array filled with module states (rotation, velocity)
     */
    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            swerveMods[0].getIntegratedState(),
            swerveMods[1].getIntegratedState(),
            swerveMods[2].getIntegratedState(),
            swerveMods[3].getIntegratedState()
        };
    }
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            swerveMods[0].getPosition(),
            swerveMods[1].getPosition(),
            swerveMods[2].getPosition(),
            swerveMods[3].getPosition()
        };
    }
    /**
     * @return An ordered array filled with the module field poses 
     */
    public Pose2d[] getModulePoses(){
        Pose2d[] modulePoses = new Pose2d[4];
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            modulePoses[i] = getPose().transformBy(new Transform2d(module.getModuleConstants().centerOffset, module.getIntegratedHeading()));
        }
        return modulePoses;
    }
    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }
    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public PIDController getXController() {
        return xController;
    }
    public PIDController getYController() {
        return yController;
    }
    // public PIDController getRotController() {
    //     return new PIDController(thetaController.getP(), thetaController.getI(), thetaController.getD());
    // }
    // public ProfiledPIDController getProfiledRotController() {
    //     return thetaController;
    // }

    public void log(){
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Drive/Heading", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Drive/X", pose.getX());
        SmartDashboard.putNumber("Drive/Y", pose.getY());
        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        // SmartDashboard.putNumber("Drive/Target Heading", Math.toDegrees(thetaController.getSetpoint().position));
        SmartDashboard.putNumber("Drive/VX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/VY", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Omega Degrees", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
        SmartDashboard.putNumber("Drive/Target VX", targetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target VY", targetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target Omega Degrees", Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
        
        for(SwerveModule module : swerveMods) {
            module.log();
        }
    }
    public void logTrajectory(Trajectory trajectory) {logTrajectory = trajectory;}
    public Trajectory getLogTrajectory() {return logTrajectory;}



    //----- Simulation
    private final BasePigeonSimCollection gyroSim = gyro.getSimCollection(); // simulate pigeon

    @Override
    public void simulationPeriodic(){
        for(SwerveModule module : swerveMods) {
            module.simulationPeriodic();
        }

        double chassisOmega = getChassisSpeeds().omegaRadiansPerSecond;
        chassisOmega = Math.toDegrees(chassisOmega);
        gyroSim.addHeading(chassisOmega*0.02);
    }

    public double getCurrentDraw(){
        double sum = 0;
        for(SwerveModule module : swerveMods) sum += module.getDriveCurrentDraw() + module.getSteerCurrentDraw();
        return sum;
    }
    /**
     * Resets the modules steer encoders to 0 and the odometry turn to 0.
     */
    public void resetMotorEncoders(){
       for(SwerveModule module : swerveMods) module.resetMotorEncoder();
       resetOdometry(
            new Pose2d(
                getPose().getTranslation(),
                new Rotation2d()
            )
        );
    }
}
