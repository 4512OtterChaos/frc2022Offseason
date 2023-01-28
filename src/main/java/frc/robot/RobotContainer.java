package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoOptions;
import frc.robot.common.OCXboxController;
import frc.robot.simulation.CargoSim;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.TeleopDriveAngle;
import frc.robot.subsystems.drivetrain.commands.TeleopDriveBasic;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotMap;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldUtil;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.*;

public class RobotContainer {
    @Log.Include
    private final SwerveDrive drivetrain = new SwerveDrive();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Vision vision = new Vision();
    private final Superstructure superstructure = new Superstructure(drivetrain, indexer, intake, shooter, vision);

    private OCXboxController driver = new OCXboxController(0);
    private OCXboxController operator = new OCXboxController(1);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final AutoOptions autoOptions = new AutoOptions(drivetrain, indexer, intake, shooter, superstructure);

    private final Field2d field = new Field2d();

    public RobotContainer(){
        configureEventBinds();

        autoOptions.submit();
        SmartDashboard.putData("Field", field);

        LiveWindow.disableAllTelemetry();
        SmartDashboard.putNumber("Shooter/RPM Offset", 0);

        Logger.configureLogging(this);
        // uncomment this line for tuning mode
        Logger.configureConfig(this);
    }

    public void periodic(){
        superstructure.periodic();
        ShotMap.setRPMOffset(SmartDashboard.getNumber("Shooter/RPM Offset", 0));
    }

    public Command getAutoCommand(){
        return autoOptions.getSelected();
    }

    public void disable(){
        drivetrain.stop();
        intake.stop();
        indexer.stop();
        shooter.stop();
        shooter.setHood(0);
    }

    public void init(boolean testMode) {
        // dynamically change binds between modes
        if(!testMode) {
            driver = new OCXboxController(0);
            operator = new OCXboxController(1);
            configureDriverBinds(driver);
            configureOperatorBinds(driver);
            configureOperatorBinds(operator);
        }
        else {
            driver = new OCXboxController(0);
            operator = new OCXboxController(1);
            configureTestBinds(driver);
        }
    }

    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
        intake.setBrakeOn(is);
        indexer.setBrakeOn(is);
    }

    private void configureEventBinds() {
        // estimate hood angle continuously before shooting
        shooter.setDefaultCommand(superstructure.autoHood());

        // count intaken cargo, rumble controllers
        new Trigger(()->indexer.getBottomSensed())
            .onTrue(superstructure.countCargo(1, driver, operator));

        // count shot cargo

        // count outtaken cargo
    }
    private void configureDriverBinds(OCXboxController controller) {
        // when no other command is using the drivetrain, we
        // pass the joysticks for forward, strafe, and angular position control
        drivetrain.setDefaultCommand(new TeleopDriveBasic(controller, drivetrain));
        //drivetrain.setDefaultCommand(new TeleopDriveAngle(controller, drivetrain));

        // push-to-change driving "speed"
        controller.rightBumper()
            .onTrue(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedMax)))
            .onFalse(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault)));

        // toggle between field-relative and robot-relative control
        controller.back().onTrue(runOnce(()->{
            drivetrain.setIsFieldRelative(!drivetrain.getIsFieldRelative());
        }));

        // reset the robot heading to 0
        controller.start().onTrue(runOnce(()->{
            drivetrain.resetOdometry(
                new Pose2d(
                    drivetrain.getPose().getTranslation(),
                    new Rotation2d()
                )
            );
        }));

        // lock the modules in a "X" alignment
        controller.x().whileTrue(run(()->{
            SwerveModuleState[] states = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            };
            drivetrain.setModuleStates(states, false, true);
        }, drivetrain));
    }
    private void configureOperatorBinds(OCXboxController controller) {

        //Clear intake and indexer
        controller.leftStick()
            .onTrue(runOnce(()->{
                intake.setVoltageOut();
                indexer.setVoltageOut();
            }, intake, indexer))
            .onFalse(runOnce(()->{
                intake.stop();
                indexer.stop();
            }, intake, indexer));
        
        // intake and automatically index cargo
        controller.rightTrigger(0.25)
            .onTrue(
                superstructure.intakeIndexCargo()
                //.deadlineWith(superstructure.rumbleCargoFull(driver, operator))
            )
            .onFalse(
                superstructure.stopIntake()
                .alongWith(
                    superstructure.stopIndexer()
                )
            );
        
        controller.b().onTrue(runOnce(()->intake.setExtended(false)));

        controller.y().onTrue(superstructure.fenderShootHigh())
        .onFalse(runOnce(()->{
            shooter.stop();
            indexer.stop();
        }, shooter, indexer));

        controller.a().onTrue(superstructure.fenderShootLow())
        .onFalse(runOnce(()->{
            shooter.stop();
            indexer.stop();
        }, shooter, indexer));

        // auto shoot high hub with pose estimation
        controller.leftTrigger(0.25).onTrue(
            superstructure.autoShoot(
                ()->driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                ()->driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                false
            ).beforeStarting(()->controller.resetLimiters())
        )
        .onFalse(
            superstructure.stopDrive()
            .alongWith(
                superstructure.stopIndexer(),
                superstructure.stopShooter()
            )
        );

        // auto shoot high hub with only vision data
        /*
        controller.leftBumper.onTrue(
            superstructure.autoShoot(
                ()->driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                ()->driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                true,
                //drivetrain.getPose().transformBy(vision.getRobotToTarget(drivetrain.getHeading())).getTranslation(),
                drivetrain.getPose().plus(
                    new Transform2d(vision.getRobotToTargetTranslation(), new Rotation2d())
                ).getTranslation(),
                ()->vision.getHasTarget()
            ).beforeStarting(()->controller.resetLimiters())
        )
        .onFalse(
            superstructure.stopDrive()
            .alongWith(
                superstructure.stopIndexer(),
                superstructure.stopShooter()
            )
        );
        */

        controller.leftBumper().onTrue(
            new InstantCommand(controller::resetLimiters)
            .andThen(new FunctionalCommand(
                ()->{
                    drivetrain.resetPathController();
                    vision.resetFilter();
                }, 
                ()->{
                    Translation2d target = vision.getFilteredRobotToTargetTranslation().plus(
                        drivetrain.getPose(vision.getLatencySeconds())
                            .orElse(drivetrain.getPose())
                            .minus(drivetrain.getPose()).getTranslation()
                    );
                    boolean hasTarget = vision.getHasTarget();
                    double dist = target.getNorm();
                    Rotation2d targetAngle = new Rotation2d(target.getX(), target.getY())
                        .plus(new Rotation2d(Math.PI))
                        .plus(drivetrain.getHeading());
                    Shooter.State targetShooterState = ShotMap.find(dist);
                    shooter.setState(targetShooterState);
                    //Drivetrain heading target to hub
                    boolean driveAtGoal = drivetrain.drive(
                        driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                        driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                        targetAngle,
                        true
                    );
                    //Indexer feed when shooter && drivetrain ready
                    if(driveAtGoal && shooter.withinTolerance() && hasTarget){
                        indexer.setVoltageFeed();
                    }
                    else{
                        indexer.stop();
                    }
                }, 
                (interrupted)->{
                    shooter.stop();
                    indexer.stop();
                    drivetrain.stop();
                },
                ()->false,
                drivetrain, shooter, indexer
            ))
        )
        .onFalse(
            superstructure.stopDrive()
            .alongWith(
                superstructure.stopIndexer(),
                superstructure.stopShooter()
            )
        );
    }

    // Manual shot tuning
    private void configureTestBinds(OCXboxController controller){
        drivetrain.setDefaultCommand(new TeleopDriveAngle(controller, drivetrain));

        // toggle between field-relative and robot-relative control
        controller.back().onTrue(runOnce(()->{
            drivetrain.setIsFieldRelative(!drivetrain.getIsFieldRelative());
        }));

        // reset the robot heading to 0
        controller.start().onTrue(runOnce(()->{
            drivetrain.resetOdometry(
                new Pose2d(
                    drivetrain.getPose().getTranslation(),
                    new Rotation2d()
                )
            );
        }));
        //Clear intake and indexer
        controller.leftStick()
            .onTrue(runOnce(()->{
                intake.setVoltageOut();
                indexer.setVoltageOut();
            }, intake, indexer))
            .onFalse(runOnce(()->{
                intake.stop();
                indexer.stop();
            }, intake, indexer));

        shooter.setDefaultCommand(new RunCommand(()->{
            shooter.setHood(SmartDashboard.getNumber("Hood MM", 0));
            shooter.setRPM(SmartDashboard.getNumber("Shooter Rpm", 0));

            //shooter.setShooterVoltage(controller.getLeftTriggerAxis()*12);
        }, shooter));

        // intake and automatically index cargo, rumble based on status
        controller.rightTrigger(0.25)
            .onTrue(
                superstructure.intakeIndexCargo()
            )
            .onFalse(
                superstructure.stopIntake()
            );

        controller.rightTrigger(0.25)
            .onTrue(runOnce(()->indexer.setVoltageFeed(), indexer))
            .onFalse(runOnce(()->indexer.stop(), indexer));
    }

    public void log(){
        Logger.updateEntries();
        drivetrain.log();
        intake.log();
        indexer.log();
        shooter.log();
        vision.log();
        
        SmartDashboard.putBoolean("Comp/Switch", compressor.getPressureSwitchValue());

        field.setRobotPose(drivetrain.getPose());
        field.getObject("vision pose").setPose(new Pose2d(
            FieldUtil.kFieldCenter.minus(vision.getRobotToTargetTranslation().rotateBy(drivetrain.getHeading())),
            new Rotation2d()
        ));
        //field.getObject("Vision Target").setPose(drivetrain.getPose().transformBy(vision.getRobotToTarget(drivetrain.getHeading())));
        field.getObject("Vision Target").setPose(drivetrain.getPose().plus(
            new Transform2d(vision.getRobotToTargetTranslation(), new Rotation2d())
        ));
        
        field.getObject("Swerve Modules").setPoses(drivetrain.getModulePoses());
        Trajectory logTrajectory = drivetrain.getLogTrajectory();
        if(logTrajectory == null) logTrajectory = new Trajectory();
        field.getObject("Trajectory").setTrajectory(logTrajectory);

        Translation2d driveTranslation = drivetrain.getPose().getTranslation();
        SmartDashboard.putNumber(
            "Shooter/DistanceInches",
            Units.metersToInches(driveTranslation.getDistance(FieldUtil.kFieldCenter))
        );

        if(!DriverStation.isFMSAttached()) NetworkTableInstance.getDefault().flush();
    }



    //----- Simulation

    private final Field2d xzField = new Field2d();
    private CargoSim cargoSimulation = new CargoSim(
        drivetrain::getPose,
        drivetrain::getChassisSpeeds,
        intake::getRPM,
        indexer::getRPM,
        shooter::getState,
        indexer::setBottomSimSensed,
        indexer::setTopSimSensed,
        field,
        xzField
    );
    public void simulationInit(){
        SmartDashboard.putData("Field XZ", xzField);
    }
    public void simulationPeriodic(){
        cargoSimulation.update();
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        sum += shooter.getCurrentDraw();
        sum += indexer.getCurrentDraw();
        sum += intake.getCurrentDraw();
        return sum;
    }
}
