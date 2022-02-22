// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.Shooter;

public class SetHoodPosition extends CommandBase {
  public final Shooter m_shooter;
  public final double m_position;
  /** Creates a new SetHoodPosition. */
  public SetHoodPosition(final Shooter shooter, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_position = position;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setHoodPosition(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.updateHoodPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(m_shooter.getHoodPosition(), m_position, 10);
  }
}
