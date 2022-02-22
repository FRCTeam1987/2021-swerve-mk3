// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;

public class StorageCapacityChecker extends CommandBase {

  private final StorageSubsystem m_storage;
  private boolean previouslyHadBallAtEntrance;

  public StorageCapacityChecker(final StorageSubsystem storage) {
    m_storage = storage;
    previouslyHadBallAtEntrance = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previouslyHadBallAtEntrance = m_storage.isBallAtEntrance();
    m_storage.spinmotorBottom(-0.5);
    m_storage.spinmotorTop(-0.75);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final boolean currentlyHasBallAtEntrance = m_storage.isBallAtEntrance();
    if (previouslyHadBallAtEntrance == false && currentlyHasBallAtEntrance == true) {
      m_storage.incrementNumofBalls();
    }
    previouslyHadBallAtEntrance = currentlyHasBallAtEntrance;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_storage.hasMaxNumberBalls();
  }
}
