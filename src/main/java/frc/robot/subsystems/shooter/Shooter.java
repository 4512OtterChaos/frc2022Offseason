// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;

public class Shooter extends SubsystemBase {
    
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(kRightMotorID);
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(kLeftMotorID);
    // private final LinearServo leftServo = new LinearServo(kLeftServoChannel, kServoLengthMM, kServoSpeedMM);
    // private final LinearServo rightServo = new LinearServo(kRightServoChannel, kServoLengthMM, kServoSpeedMM);

    private State targetState = new State();

    public Shooter() {
        setupFlywheel(true);
        
        // leftServo.setMaxPos(kServoMaxMM);
        // rightServo.setMaxPos(kServoMaxMM);
    }

    private void setupFlywheel(boolean init){
        if(init){
            rightMotor.configAllSettings(kFlywheelConfig);
            leftMotor.configAllSettings(kFlywheelConfig);
        }
        rightMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.enableVoltageCompensation(true);
        leftMotor.setNeutralMode(NeutralMode.Coast);
        leftMotor.enableVoltageCompensation(true);
        rightMotor.setInverted(kRightMotorInversion);
        leftMotor.follow(rightMotor);
        leftMotor.setInverted(kLeftMotorInversion);
        TalonUtil.configStatusFollower(leftMotor);
        TalonUtil.configStatusCurrent(rightMotor);
    }
    
    @Override
    public void periodic() {
        // estimate current servo position
        // leftServo.updateCurPos();
        // rightServo.updateCurPos();

        // check if the motors had an oopsie, reapply settings
        if(rightMotor.hasResetOccurred() || leftMotor.hasResetOccurred()){
            setupFlywheel(false);
        }
        
    }

    public void setRPM(double rpm){
        double radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
        double nativeVelocity = TalonUtil.radiansToVelocity(radiansPerSecond, 1);
        if(rpm == 0){
            rightMotor.set(0);
        }
        else{
            rightMotor.set(
                ControlMode.Velocity, nativeVelocity,
                DemandType.ArbitraryFeedForward, kFF.calculate(radiansPerSecond)/kVoltageSaturation
            );
        }
    }
    // public void setHood(double extensionMM){
    //     leftServo.setPosition(extensionMM);
    //     rightServo.setPosition(extensionMM);
    // }
    public void setShooterVoltage(double volts){
        rightMotor.set(volts/kVoltageSaturation);   

    }
    public void setState(State state){
        targetState = state;
        setRPM(state.rpm);
        // setHood(state.hoodMM);
    }
    public void stop(){
        setState(new State());
    }

    public State getState(){
        return new State(
            TalonUtil.velocityToRotations(rightMotor.getSelectedSensorVelocity(), 1)*60
            // rightServo.getPosition()
        );
    }
    public State getTargetState(){return targetState;}
    public boolean withinTolerance(){
        return getState().withinTolerance(getTargetState());
    }

    public void log(){
        State state = getState();
        SmartDashboard.putNumber("Shooter/Velocity", rightMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter/TargetVelocity", rightMotor.getControlMode() == ControlMode.Velocity ?
            rightMotor.getClosedLoopTarget()
            :
            0
        );
        SmartDashboard.putNumber("Shooter/RPM", state.rpm);
        // SmartDashboard.putNumber("Shooter/HoodMM", state.hoodMM);
    }
    
    /**
     * Represents a state of flywheel rpm and hood angle achievable by the shooter.
     * Shooter states can be transitioned from by using {@link #interpolate(State, double)}.
     */
    public static class State implements Interpolatable<Shooter.State>{
        public final double rpm;
        // public final double hoodMM;
        
        public State(double rpm){
            this.rpm = rpm;
        }
        public State(){
            this(0);
        }
        public State plus(State other){
            return new State(this.rpm+other.rpm);
        }
        public boolean withinTolerance(State other){
            return Math.abs(other.rpm - rpm) <= kToleranceRPM;
        }
        
        @Override
        public State interpolate(State endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                double newRPM = MathUtil.interpolate(rpm, endValue.rpm, t);
                // double newHoodMM = MathUtil.interpolate(hoodMM, endValue.hoodMM, t);
                return new State(newRPM);
            }
        }
    }



    // Simulation
    private final TalonFXSimCollection rightMotorSim = new TalonFXSimCollection(rightMotor);
    private final TalonFXSimCollection leftMotorSim = new TalonFXSimCollection(leftMotor);
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kFF.kv, kFF.ka),
        DCMotor.getFalcon500(2),
        1,
        VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(5))
    );

    @Override
    public void simulationPeriodic(){
        flywheelSim.setInputVoltage(rightMotorSim.getMotorOutputLeadVoltage());
        flywheelSim.update(0.02);

        double flywheelMotorVelocityNative = TalonUtil.radiansToVelocity(
            flywheelSim.getAngularVelocityRadPerSec(),
            1
        );
        rightMotorSim.setIntegratedSensorVelocity((int)flywheelMotorVelocityNative);
        rightMotorSim.setSupplyCurrent(flywheelSim.getCurrentDrawAmps()/4);
        leftMotorSim.setIntegratedSensorVelocity((int)flywheelMotorVelocityNative);
        leftMotorSim.setSupplyCurrent(flywheelSim.getCurrentDrawAmps()/4);

        rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    }

    public double getCurrentDraw(){
        return leftMotor.getSupplyCurrent() + rightMotor.getSupplyCurrent();
    }
}
