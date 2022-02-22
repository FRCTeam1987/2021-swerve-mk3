// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final WPI_TalonFX intakeTalon;
  private final Solenoid intakeSolenoid;
  public IntakeSubsystem() {
    intakeTalon = new WPI_TalonFX(Constants.COLLECTOR_MOTOR);
    intakeTalon.configFactoryDefault();
    intakeTalon.setNeutralMode(NeutralMode.Coast);
    intakeSolenoid = new Solenoid(Constants.Solenoid.intakeSolenoid);
  }

  public void stop() {
    intakeTalon.set(0);
  }

  public void deploy() {
    intakeSolenoid.set(true);
  }

  public void stow() {
    intakeSolenoid.set(false);
  }

  public void toggle() {
    intakeSolenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  public void spinmotor(double spinPercent) {
    intakeTalon.set(spinPercent);
  }
}
