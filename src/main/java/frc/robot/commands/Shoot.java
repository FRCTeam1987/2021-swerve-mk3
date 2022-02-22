// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StorageSubsystem;

public class Shoot extends CommandBase {

  private final double tempRpmSetpoint = 1000;
  private final double m_feedRateLower = 0.5;
  private final double m_feedRateUpper = 0.75;

  private final Shooter m_shooterSubsystem;
  private final StorageSubsystem m_storageSubsystem;

  /** Creates a new Shoot. */
  public Shoot(final Shooter shooterSubsystem, final StorageSubsystem storageSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_storageSubsystem = storageSubsystem;
    addRequirements(m_shooterSubsystem, m_storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setRPM(tempRpmSetpoint);  // TODO get from limelight
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!Util.isWithinTolerance(m_shooterSubsystem.getRPM(), tempRpmSetpoint, 50)) {
      return;
    }
    m_storageSubsystem.spinmotorBottom(-m_feedRateLower);
    m_storageSubsystem.spinmotorTop(m_feedRateUpper);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_storageSubsystem.stop();
    m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
