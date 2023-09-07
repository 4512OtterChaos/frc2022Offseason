// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenixpro.controls.CoastOut;
import com.ctre.phoenixpro.hardware.TalonFX;

public class Robot extends TimedRobot {

    TalonFX leftMotor = new TalonFX(0);

    TalonFX[] otherMotors = {
        new TalonFX(1)
    };

    PowerDistribution pdp = new PowerDistribution();

    DoubleLogEntry supplyVoltageLog;
    DoubleLogEntry supplyCurrentLog;
    DoubleLogEntry statorVoltageLog;
    DoubleLogEntry statorCurrentLog;
    DoubleLogEntry pdpCurrentLog;

    public Robot() {
        super(0.01);
    }

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // configure leftMotor

        // configure otherMotors
        for(var motor : otherMotors) {
            motor.setControl(new CoastOut());
        }

        DataLogManager.start();
        var log = DataLogManager.getLog();
        supplyVoltageLog = new DoubleLogEntry(log, "supplyVoltage");
        supplyCurrentLog = new DoubleLogEntry(log, "supplyCurrent");
        statorVoltageLog = new DoubleLogEntry(log, "statorVoltage");
        statorCurrentLog = new DoubleLogEntry(log, "statorCurrent");
        pdpCurrentLog = new DoubleLogEntry(log, "pdpCurrent");
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        pdpCurrentLog.append(pdp.getCurrent(0));
        
    }
    
    @Override
    public void autonomousInit() {
        brakingTest().schedule();
    }

    public CommandBase brakingTest() {
        return sequence();
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {
    }
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void disabledInit() {
    }
    
    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void simulationInit(){
    }
    @Override
    public void simulationPeriodic(){
    }
    
    @Override
    public void testInit() {
    }
    
    @Override
    public void testPeriodic() {}
}
