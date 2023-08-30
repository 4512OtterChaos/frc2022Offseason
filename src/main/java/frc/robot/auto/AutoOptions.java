// package frc.robot.auto;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.drivetrain.SwerveDrive;
// import frc.robot.subsystems.indexer.Indexer;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.shooter.Shooter;

// public class AutoOptions {
    
//     // list of choosable commands that decides what is run in auto
//     private SendableChooser<Command> autoOptions = new SendableChooser<>();
//     private PathPlannerTrajectory tripleRightTrajectory = PathPlanner.loadPath("TripleRight1", 1, 1);

//     public AutoOptions(SwerveDrive drivetrain, Indexer indexer, Intake intake, Shooter shooter, Superstructure superstructure){

//         autoOptions.setDefaultOption("Nothing",
//             new InstantCommand(()->drivetrain.stop(), drivetrain)
//         );
        
//         autoOptions.addOption("TripleRight", 
//             superstructure.autoShoot(1.5)
//             .beforeStarting(()->{
//                 drivetrain.resetOdometry(new Pose2d(
//                     tripleRightTrajectory.getInitialPose().getTranslation(), 
//                     tripleRightTrajectory.getInitialState().holonomicRotation));
//             })
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "TripleRight1", 
//                     AutoConstants.kMediumSpeedConfig,
//                     false
//                 ) 
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
//             .andThen(superstructure.autoShoot(3))
//             .andThen(superstructure.stop())
//         );

//         autoOptions.addOption("FenderTripleRight", 
//             superstructure.fenderShootHigh(2)
//             .andThen(()->{
//                 shooter.stop();
//                 indexer.stop();
//             }, indexer, shooter)
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "TripleRightFender1", 
//                     AutoConstants.kSlowSpeedConfig,
//                     true
//                 ) 
//                 .deadlineWith(superstructure.intakeIndexCargo())
//                 .andThen(()->drivetrain.stop(), drivetrain)
//             )
//             .andThen(
//                 superstructure.fenderShootHigh(3)
//             )
//             .andThen(superstructure.stop())
//         );
        
//         autoOptions.addOption("DoubleLeft",
//             new InstantCommand(
//                 ()->intake.setExtended(true), 
//                 intake
                
//                 )
//             .andThen(new WaitCommand(2))
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "DoubleLeft1", 
//                     AutoConstants.kSlowSpeedConfig,
//                     true
//                 ) 
//                 .deadlineWith(
//                     superstructure.intakeIndexCargo(),
//                     superstructure.autoHood()
//                 )
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(superstructure.autoShoot(3))
//             .andThen(superstructure.stop())
//         );

//         autoOptions.addOption("DoubleLeft but troll", 
//             new InstantCommand(
//                 ()->intake.setExtended(true), 
//                 intake
                
//                 )
//             .andThen(new WaitCommand(2))    
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "DoubleLeft1", 
//                     AutoConstants.kSlowSpeedConfig,
//                     true
//                 ) 
//                 .deadlineWith(
//                     superstructure.intakeIndexCargo(),
//                     superstructure.autoHood()
//                 )
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(superstructure.autoShoot(4))
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "DoubleLeftTroll2", 
//                     AutoConstants.kSlowSpeedConfig, 
//                     false
//                 )
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
            
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.dumpCargo()
//                 .withTimeout(3)
//             )

            
//         );

//         autoOptions.addOption("FenderDoubleLeft",
//             new OCSwerveFollower(
//                 drivetrain, 
//                 "DoubleLeft1", 
//                 AutoConstants.kSlowSpeedConfig,
//                 true
//             ) 
//             .deadlineWith(superstructure.intakeIndexCargo())
//             .andThen(new OCSwerveFollower(
//                 drivetrain, 
//                 "FenderDoubleLeft2", 
//                 AutoConstants.kSlowSpeedConfig,
//                 false
//                 )
//                 .andThen(()->drivetrain.stop(), drivetrain)
//             )
//             .andThen(
//                 superstructure.fenderShootHigh(3)
//             )
//             .andThen(superstructure.stop())
           
//         );
//         /*
//         autoOptions.addOption("-1auto",
//             new WaitCommand(2)
//             .deadlineWith(superstructure.dumpCargo())
            
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "sabotage", 
//                     AutoConstants.kMediumSpeedConfig, 
//                     true
//                 )
//             )
            
//             .andThen(superstructure.stopDrive())
            
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "sabotage2", 
//                     AutoConstants.kMediumSpeedConfig, 
//                     false)
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
            
//             .andThen(
//                 superstructure.autoShoot(2.5)
//             )
            
//         );
//         */
//         /*
//         autoOptions.addOption("QuintupleLeft", 
//             new OCSwerveFollower(
//                 drivetrain, 
//                 "QuintetLeft1", 
//                 AutoConstants.kMediumSpeedConfig, 
//                 true
//                 )
//             .deadlineWith(superstructure.intakeIndexCargo())
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.autoShoot(2.5)
//             )
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "QuintetLeft2", 
//                     AutoConstants.kMediumSpeedConfig, 
//                     false
//                 )
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
            
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.autoShoot(2.5)
//             )
//             .andThen(
//                 new OCSwerveFollower(
//                 drivetrain, 
//                 "QuintetLeft3", 
//                 AutoConstants.kMediumSpeedConfig, 
//                 false)
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
            
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.autoShoot(2.5)
//             )
            
            
//         );
//         */
//         /*
//         autoOptions.addOption("FenderQuintupleRight",
//             superstructure.fenderShootHigh(1.75)
//             .andThen(()->{
//                 shooter.stop();
//                 indexer.stop();
//             }, indexer, shooter)
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "TripleRightFender1", 
//                     AutoConstants.kMediumSpeedConfig,
//                     true
//                 ) 
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.fenderShootHigh(1.5)
//             )
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "TripleRightFender2", 
//                     AutoConstants.kMediumSpeedConfig,
//                     false
//                 )
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 new WaitCommand(0.75)
//                 .deadlineWith(superstructure.intakeIndexCargo())
//             )
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "QuintupleRightFender3", 
//                     AutoConstants.kFastSpeedConfig,
//                     false
//                 ) 
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.fenderShootHigh(3)
//             )
//             .andThen(superstructure.stop())

//         );
//         */
//         autoOptions.addOption("QuintupleRight",
            
//             superstructure.autoShoot(2)
//             .beforeStarting(()->{
//                 drivetrain.resetOdometry(new Pose2d(
//                     tripleRightTrajectory.getInitialPose().getTranslation(), 
//                     tripleRightTrajectory.getInitialState().holonomicRotation));
//             })
//             .alongWith(
//                 new InstantCommand(()-> intake.setExtended(true), 
//                     intake
//                 )
//             )
//             .andThen(()->{
//                 shooter.stop();
//                 indexer.stop();
//             }, indexer, shooter)
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "TripleRight1", 
//                     AutoConstants.kMediumSpeedConfig,
//                     false
//                 ) 
//                 .deadlineWith(
//                     superstructure.intakeIndexCargo(drivetrain::getLinearVelocity),
//                     superstructure.autoHood()
//                 )
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.autoShoot(2.5)
//             )
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "QuintetRight2", 
//                     AutoConstants.kMediumSpeedConfig,
//                     false
//                 )
//                 .deadlineWith(
//                     superstructure.intakeIndexCargo(drivetrain::getLinearVelocity)
//                 )
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 new WaitCommand(0.5)
//                 .deadlineWith(superstructure.intakeIndexCargo(drivetrain::getLinearVelocity))
//             )
//             .andThen(
//                 new OCSwerveFollower(
//                     drivetrain, 
//                     "QuintetRight3", 
//                     AutoConstants.kFastSpeedConfig,
//                     false
//                 )
//                 .deadlineWith(
//                     superstructure.autoHood()
//                 )
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//             .andThen(
//                 superstructure.autoShoot(3)
//             )
//             .andThen(superstructure.stop())

//         );

//         //Don't use, no odometry; only as last resort 
//         autoOptions.addOption("TaxiLastResort",
//             new WaitCommand(2)
//             .deadlineWith(new RunCommand(()->drivetrain.drive(0.6, 0, 0, false), drivetrain))
//             .andThen(()->drivetrain.stop(), drivetrain)
//         );

//         autoOptions.addOption("ShootThenTaxiLastResort",
//             superstructure.autoShoot(2)
//             .andThen(()->{
//                 shooter.stop();
//                 indexer.stop();
//             }, indexer, shooter)
//             .andThen(
//                 new WaitCommand(2)
//                 .deadlineWith(new RunCommand(()->drivetrain.drive(2, 0, 0, false), drivetrain))
//             )
//             .andThen(()->drivetrain.stop(), drivetrain)
//         );
        
//     }

//     // Network Tables
//     public Command getSelected(){
//         return autoOptions.getSelected();
//     }

//     public void submit(){
//         SmartDashboard.putData("Auto Options", autoOptions);
//     }
// }
