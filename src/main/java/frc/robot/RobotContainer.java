// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController m_controller = new XboxController(DRIVER_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getY(GenericHID.Hand.kLeft)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getX(GenericHID.Hand.kLeft)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getX(GenericHID.Hand.kRight)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    // TODO remove this
    m_drivetrainSubsystem.followPathCommand(true, "2m Figure 8"
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Drive straight for 3 meters!
    // return m_drivetrainSubsystem.followPathCommand(true,
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   new Pose2d(3, 0, Rotation2d.fromDegrees(0))
    // );

    // Drive straight for 2 meters and turn left 90 degrees!
    return m_drivetrainSubsystem.followPathCommand(true, "2m Figure 8" //Path not rotating, ?

      // new Pose2d(0.25, 0, Rotation2d.fromDegrees(11.25)),
      // new Pose2d(0.5, 0, Rotation2d.fromDegrees(22.5)),
      // new Pose2d(0.75, 0, Rotation2d.fromDegrees(33.75)),
      // new Pose2d(1, 0, Rotation2d.fromDegrees(45)),
      // new Pose2d(1.25, 0, Rotation2d.fromDegrees(56.25)),
      // new Pose2d(1.5, 0, Rotation2d.fromDegrees(67.5)),
      // new Pose2d(1.75, 0, Rotation2d.fromDegrees(78.75)),
    );

    // Drive forward left, center, right, and center.
    // return m_drivetrainSubsystem.followPathCommand(true,
    //   new Pose2d(0, 0, new Rotation2d()),
    //   new Pose2d(0.5, 0.5, new Rotation2d()),
    //   new Pose2d(1, 0, new Rotation2d()),
    //   new Pose2d(1.5, -0.5, new Rotation2d()),
    //   new Pose2d(2, 0, new Rotation2d())
    // );
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
