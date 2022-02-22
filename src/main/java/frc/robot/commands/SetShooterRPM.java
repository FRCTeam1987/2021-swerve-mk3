// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends CommandBase {

  private final Shooter m_shooter;
  private final double m_rpmSetpoint;

  /** Creates a new SetShooterRPM. */
  public SetShooterRPM(final Shooter shooter, final double rpm) {
    m_shooter = shooter;
    m_rpmSetpoint = rpm;
    addRequirements(m_shooter);
  }

  public SetShooterRPM(final Shooter shooter) {
    this(shooter, -1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPM(m_rpmSetpoint == -1 ? 
      SmartDashboard.getNumber("RPM Setpoint", 0.0)
      : m_rpmSetpoint
    );
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    final double rpm = m_shooter.getRPM();
    SmartDashboard.putNumber("RPM", rpm);
    SmartDashboard.putNumber("RPM Percent", rpm / 6380.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_rpmSetpoint == 0.0) {
      return true;
    }
    return Util.isWithinTolerance(m_shooter.getRPM(), m_rpmSetpoint, 200);
  }
}
