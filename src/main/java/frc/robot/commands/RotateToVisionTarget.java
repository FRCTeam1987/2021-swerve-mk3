// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.*;

import java.util.OptionalDouble;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToVisionTarget extends ProfiledPIDCommand {
  private final VisionSubsystem m_visionSubsystem;
  /** Creates a new RotateToVisionTarget. */
  public RotateToVisionTarget(final DrivetrainSubsystem drivetrain, final VisionSubsystem vision) {
    super(
      // The ProfiledPIDController used by the command
      createThetaController(),
      // This should return the measurement
      () -> {
        final OptionalDouble angleToTarget = vision.getAngleToTarget();
        if (angleToTarget.isEmpty()) {
          return 0.0;
        }
        return angleToTarget.getAsDouble();
      },
      // This should return the goal (can also be a constant)
      () -> new TrapezoidProfile.State(0, 0),
      // This uses the output
      (output, setpoint) -> {
        drivetrain.drive(new ChassisSpeeds(0, 0, output));
      });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_visionSubsystem = vision;
    addRequirements(drivetrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_visionSubsystem.isOnTarget();
  }

  private static ProfiledPIDController createThetaController() {
    final ProfiledPIDController thetaController = new ProfiledPIDController(
      // The PID gains
      DRIVETRAIN_PTHETA_CONTROLLER,
      0,
      0,
      // The motion profile constraints
      new TrapezoidProfile.Constraints(
        DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED
      )
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return thetaController;
  }
}
