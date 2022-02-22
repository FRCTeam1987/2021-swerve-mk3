package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;

public class StorageSpinBottom extends CommandBase {
  private StorageSubsystem storageSubsystem;
  private double spinSpeed;

  public StorageSpinBottom(StorageSubsystem storageSubsystem2, double spinSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storageSubsystem = storageSubsystem2;
    this.spinSpeed = spinSpeed;
    addRequirements(this.storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.storageSubsystem.spinmotorBottom(-spinSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.storageSubsystem.spinmotorBottom(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}