// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.lib.LinearServo;

public class Shooter extends SubsystemBase {

  private final WPI_TalonFX m_flywheel;
  private final WPI_TalonFX m_flywheelFollower;
  private final LinearServo m_linearServoLeft;
  private final LinearServo m_linearServoRight;

  /** Creates a new Shooter. */
  public Shooter() {
    m_linearServoLeft = new LinearServo(0, 100, 32);
    m_linearServoRight = new LinearServo(1, 100, 32);

    m_flywheel = new WPI_TalonFX(Constants.SHOOTER_MOTOR_LEFT);
    m_flywheelFollower = new WPI_TalonFX(Constants.SHOOTER_MOTOR_RIGHT);

    m_flywheel.configFactoryDefault();
    m_flywheel.configOpenloopRamp(1.5);
    m_flywheel.setNeutralMode(NeutralMode.Coast);
    m_flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    m_flywheel.configClosedloopRamp(0.5);
    m_flywheel.config_kF(0, 0.052); //get started umf (increases the actual base rpm exponentially or something) was.052 old BAT, new bat .0498, then was .055 for 3550 rpm, then .052
    m_flywheel.config_kP(0, 0.3); //p = push and oscillating once it gets there
    m_flywheel.config_kI(0, 0.0);
    m_flywheel.config_kD(0, 0.0); //d =  dampening for the oscillation

    m_flywheelFollower.configFactoryDefault();
    m_flywheelFollower.configOpenloopRamp(1.5);
    m_flywheelFollower.setNeutralMode(NeutralMode.Coast);
    m_flywheelFollower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    m_flywheelFollower.configClosedloopRamp(0.5);
    m_flywheelFollower.follow(m_flywheel);
    m_flywheelFollower.setInverted(TalonFXInvertType.OpposeMaster);
    m_flywheelFollower.config_kF(0, 0.052); //get started umf (increases the actual base rpm exponentially or something) was.052 old BAT, new bat .0498, then was .055 for 3550 rpm, then .052
    m_flywheelFollower.config_kP(0, 0.3); //p = push and oscillating once it gets there
    m_flywheelFollower.config_kI(0, 0.0);
    m_flywheelFollower.config_kD(0, 0.0); //d =  dampening for the oscillation

    SmartDashboard.putNumber("RPM Setpoint", 0.0);
    SmartDashboard.putNumber("RPM", 0.0);

    SmartDashboard.putNumber("LinearServo Setpoint", 0.0);
    SmartDashboard.putNumber("LinearServo Length", 0.0);

  } 

  public double getRPM(){
    return m_flywheel.getSensorCollection().getIntegratedSensorVelocity() / 2048.0 * 600.0;
  }

  public void setRPM(final double rpm) {
    double velocity = rpm * 2048.0 / 600.0; //1,000ms per sec, but robot cares about per 100ms, so then 60 sec/min 
    m_flywheel.set(TalonFXControlMode.Velocity, velocity);
  }
  public void setHoodPosition(final double position) {
    m_linearServoRight.setPosition(position);
    m_linearServoLeft.setPosition(position);
  }

  public double getHoodPosition() {
    return m_linearServoLeft.getPosition();
  }

  public void stop() {
    m_flywheel.set(0);
  }

  public void updateHoodPosition() {
    m_linearServoLeft.updateCurPos();
    m_linearServoRight.updateCurPos();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("RPM", getRPM());
  }
}
