// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Limelight;

public class VisionSubsystem extends SubsystemBase {

  private static final double TARGET_HEIGHT = Units.inchesToMeters(98.25);
  private static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(22);
  private static final Rotation2d LIMELIGHT_ANGLE = Rotation2d.fromDegrees(30);

  private static final Rotation2d ALIGNMENT_TOLERANCE = Rotation2d.fromDegrees(2);

  private final Limelight m_limelight;
  private final DrivetrainSubsystem m_driveTrainSubsystem;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(final DrivetrainSubsystem drivetrain) {
    m_driveTrainSubsystem = drivetrain;
    m_limelight = new Limelight();
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  }

  public boolean canSeeTarget() {
    return m_limelight.canSeeTarget();
  }

  public OptionalDouble getAngleToTarget() {
    if (canSeeTarget()) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(m_limelight.getTargetX());
  }

  public boolean isOnTarget() {
    OptionalDouble angleToTarget = getAngleToTarget();
    if (angleToTarget.isEmpty()) {
      return false;
    }
    return Math.abs(angleToTarget.getAsDouble()) < ALIGNMENT_TOLERANCE.getDegrees();
  }

  public void setCamMode(final Limelight.CamMode mode) {
    m_limelight.setCamMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
