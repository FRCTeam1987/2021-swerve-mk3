package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;

public class StorageSpinTop extends CommandBase {
  private StorageSubsystem storageSubsystem;
  private double spinSpeed;

  public StorageSpinTop(StorageSubsystem storageSubsystem, double spinSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storageSubsystem = storageSubsystem;
    this.spinSpeed = spinSpeed;
    addRequirements(this.storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.storageSubsystem.spinmotorTop(spinSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.storageSubsystem.spinmotorTop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}