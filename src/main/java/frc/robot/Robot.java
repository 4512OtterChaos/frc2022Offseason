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

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.CoastOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

public class Robot extends TimedRobot {

    TalonFX leftMotor = new TalonFX(8);

    TalonFX[] otherMotors = {
        new TalonFX(9)
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
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotor.getConfigurator().apply(config);
        leftMotor.getSupplyVoltage().setUpdateFrequency(200, 0.1);
        leftMotor.getSupplyCurrent().setUpdateFrequency(200, 0.1);
        leftMotor.getStatorCurrent().setUpdateFrequency(200, 0.1);
        leftMotor.getDutyCycle().setUpdateFrequency(200, 0.1);
        // configure otherMotors
        for(var motor : otherMotors) {
            motor.setControl(new CoastOut());
            config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            motor.getConfigurator().apply(config);
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

        statorVoltageLog.append(leftMotor.get()*leftMotor.getSupplyVoltage().getValue());
        statorCurrentLog.append(leftMotor.getStatorCurrent().getValue());
        supplyVoltageLog.append(leftMotor.getSupplyVoltage().getValue());
        supplyCurrentLog.append(leftMotor.getSupplyCurrent().getValue());
        pdpCurrentLog.append(pdp.getCurrent(2));
        
    }
    
    @Override
    public void autonomousInit() {
        // brakingTest().schedule();
        regenTest().schedule();
    }

    public CommandBase brakingTest() {
        return sequence(
            runOnce(()->leftMotor.set(0.5)),
            waitSeconds(2),
            runOnce(()->leftMotor.set(0)),
            waitSeconds(1.5),
            runOnce(()->leftMotor.set(0.5)),
            waitSeconds(2),
            runOnce(()->leftMotor.set(0.05)),
            waitSeconds(1.5),
            runOnce(()->leftMotor.set(0.5)),
            waitSeconds(2),
            runOnce(()->leftMotor.set(-0.05)),
            waitSeconds(1.5),
            runOnce(()->leftMotor.set(0.5)),
            waitSeconds(2),
            runOnce(()->leftMotor.set(-0.5)),
            waitSeconds(2),
            runOnce(()->leftMotor.set(0))
        );
    }

    public CommandBase regenTest() {
        return sequence(
            runOnce(()->leftMotor.set(0)),
            waitSeconds(6),
            runOnce(()->leftMotor.set(0.1)),
            waitSeconds(4),
            runOnce(()->leftMotor.set(-0.1)),
            waitSeconds(4)
        );
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
