// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Proof of concept.
 * TODO dynamic angle - ex. angle to vision target
 * TODO angle relative to current robot angle
 * TODO proper min and max move bounds, pid not tuned
 */

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToAngle extends PIDCommand {
  /** Creates a new RotateToAngle. */
  public RotateToAngle(final DrivetrainSubsystem drivetrain, final double angleDegrees) {
    super(
        // The controller that the command will use
        new PIDController(4.0, 0.5, 0),
        // This should return the measurement
        () -> drivetrain.getPose().getRotation().getRadians(),
        // This should return the setpoint (can also be a constant)
        () -> Rotation2d.fromDegrees(angleDegrees).getRadians(),
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.drive(new ChassisSpeeds(0, 0, output));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(
      Rotation2d.fromDegrees(1.5).getRadians(),
      Rotation2d.fromDegrees(5).getRadians()
    );
    getController().enableContinuousInput(-Math.PI, Math.PI);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
