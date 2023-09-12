// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
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

    TalonFX mainMotor = new TalonFX(8);

    TalonFX counterMotor = new TalonFX(9);

    TalonFX[] otherMotors = {
        new TalonFX(0),
        new TalonFX(1),
        new TalonFX(2),
        new TalonFX(3),
        new TalonFX(4),
        new TalonFX(5),
        new TalonFX(6),
        new TalonFX(7),
    };

    PowerDistribution pdp = new PowerDistribution();

    StringLogEntry testNameLog;
    DoubleLogEntry supplyVoltageLog;
    DoubleLogEntry supplyCurrentLog;
    DoubleLogEntry pdpCurrentLog;
    DoubleLogEntry statorCurrentLog;
    DoubleLogEntry dutyCycleLog;
    DoubleLogEntry appliedVoltageLog;
    DoubleLogEntry velocityLog;
    StringLogEntry bridgeOutputLog;
    IntegerLogEntry faultFieldLog;
    BooleanLogEntry statorCurrLimitLog;
    BooleanLogEntry unstableVoltageLog;

    DoubleLogEntry counterAppliedVoltageLog;

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
        mainMotor.getConfigurator().apply(config);
        mainMotor.getSupplyVoltage().setUpdateFrequency(200, 0.1);
        mainMotor.getSupplyCurrent().setUpdateFrequency(200, 0.1);
        mainMotor.getStatorCurrent().setUpdateFrequency(200, 0.1);
        mainMotor.getDutyCycle().setUpdateFrequency(200, 0.1);
        mainMotor.getVelocity().setUpdateFrequency(200, 0.1);
        // configure counterMotor
        config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        counterMotor.getDutyCycle().setUpdateFrequency(200, 0.1);
        counterMotor.getConfigurator().apply(config);
        counterMotor.setControl(new CoastOut());
        // configure other motors
        for(var motor : otherMotors) {
            config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            motor.getPosition().setUpdateFrequency(5, 0.1);
            motor.getVelocity().setUpdateFrequency(5, 0.1);
            motor.getConfigurator().apply(config);
            motor.setControl(new CoastOut());
        }

        var log = DataLogManager.getLog();
        testNameLog = new StringLogEntry(log, "testName");
        supplyVoltageLog = new DoubleLogEntry(log, "supplyVoltage");
        supplyCurrentLog = new DoubleLogEntry(log, "supplyCurrent");
        pdpCurrentLog = new DoubleLogEntry(log, "pdpCurrent");
        statorCurrentLog = new DoubleLogEntry(log, "statorCurrent");
        dutyCycleLog = new DoubleLogEntry(log, "dutyCycle");
        appliedVoltageLog = new DoubleLogEntry(log, "appliedVoltage");
        velocityLog = new DoubleLogEntry(log, "velocity");
        bridgeOutputLog = new StringLogEntry(log, "bridgeOutput");
        faultFieldLog = new IntegerLogEntry(log, "faultField");
        statorCurrLimitLog = new BooleanLogEntry(log, "statorCurrLimitFlag");
        unstableVoltageLog = new BooleanLogEntry(log, "unstableVoltageFlag");

        counterAppliedVoltageLog = new DoubleLogEntry(log, "counterAppliedVoltage");
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        double supplyVolts = mainMotor.getSupplyVoltage().getValue();
        supplyVoltageLog.append(supplyVolts);
        supplyCurrentLog.append(mainMotor.getSupplyCurrent().getValue());
        pdpCurrentLog.append(pdp.getCurrent(2));
        statorCurrentLog.append(mainMotor.getStatorCurrent().getValue());
        dutyCycleLog.append(mainMotor.get());
        appliedVoltageLog.append(mainMotor.get() * supplyVolts);
        velocityLog.append(mainMotor.getVelocity().getValue());
        bridgeOutputLog.append(mainMotor.getBridgeOuput().getValue().toString());
        faultFieldLog.append(mainMotor.getFaultField().getValue());
        statorCurrLimitLog.append(mainMotor.getFault_StatorCurrLimit().getValue());
        unstableVoltageLog.append(mainMotor.getFault_UnstableSupplyV().getValue());

        counterAppliedVoltageLog.append(counterMotor.getDutyCycle().getValue() * supplyVolts);
    }
    
    @Override
    public void autonomousInit() {
        sequence(
            brakingTest(),
            waitSeconds(5),
            dutyCycleTest(),
            waitSeconds(5),
            regenTest()
        ).schedule();
    }

    public CommandBase brakingTest() {
        final double high = 0.75;
        final double highTime = 2;
        final double low = 0.05;
        final double lowTime = 1.5;
        return sequence(
            runOnce(()->testNameLog.append("brakingTest")),
            runOnce(()->mainMotor.set(high)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.set(0)),
            waitSeconds(lowTime),
            runOnce(()->mainMotor.set(high)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.set(low)),
            waitSeconds(lowTime),
            runOnce(()->mainMotor.set(high)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.set(-low)),
            waitSeconds(lowTime),
            runOnce(()->mainMotor.set(high)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.set(-high)),
            waitSeconds(3),
            runOnce(()->mainMotor.set(0)),

            runOnce(()->testNameLog.append("idle"))
        );
    }

    public CommandBase regenTest() {
        final double high = 0.7;
        final double low = 0.05;
        final double waitTime = 2;
        return sequence(
            runOnce(()->testNameLog.append("regenTest1")),
            runOnce(()->mainMotor.set(high)),
            waitSeconds(waitTime),
            runOnce(()->counterMotor.set(low)),
            waitSeconds(waitTime),
            runOnce(()->counterMotor.set(-low)),
            waitSeconds(waitTime),
            runOnce(()->mainMotor.set(0)),
            runOnce(()->counterMotor.set(0)),

            waitSeconds(waitTime),

            runOnce(()->testNameLog.append("regenTest2")),
            runOnce(()->counterMotor.set(high)),
            waitSeconds(waitTime),
            runOnce(()->mainMotor.set(low)),
            waitSeconds(waitTime),
            runOnce(()->mainMotor.set(-low)),
            waitSeconds(waitTime),
            runOnce(()->mainMotor.set(0)),
            runOnce(()->counterMotor.set(0)),

            runOnce(()->testNameLog.append("idle"))
        );
    }

    public CommandBase dutyCycleTest() {
        final double high = 0.75;
        final double highTime = 2;
        final double lowTime = 1.5;
        return sequence(
            runOnce(()->testNameLog.append("dutyCycleTest1")),
            runOnce(()->mainMotor.set(high)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.set(0)),
            waitSeconds(lowTime),
            runOnce(()->mainMotor.set(high)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.set(-high)),
            waitSeconds(3),
            runOnce(()->mainMotor.set(0)),

            waitSeconds(3),

            runOnce(()->testNameLog.append("dutyCycleTest2")),
            runOnce(()->mainMotor.setVoltage(high*12)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.setVoltage(0)),
            waitSeconds(lowTime),
            runOnce(()->mainMotor.setVoltage(high*12)),
            waitSeconds(highTime),
            runOnce(()->mainMotor.setVoltage(-high*12)),
            waitSeconds(3),
            runOnce(()->mainMotor.setVoltage(0)),

            runOnce(()->testNameLog.append("idle"))
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
