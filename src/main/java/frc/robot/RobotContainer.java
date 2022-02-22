// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.BallCounterReset;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DeployCollector;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.Shoot;
import frc.robot.commands.StorageCapacityChecker;
import frc.robot.commands.StorageSpin;
import frc.robot.commands.StorageSpinBottom;
import frc.robot.commands.StorageSpinTop;
import frc.robot.commands.StowCollector;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StorageSubsystem;

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

  private final StorageSubsystem m_storageSubsystem = new StorageSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final Shooter m_shooter = new Shooter();
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
            () -> -modifyAxis(m_controller.getX(GenericHID.Hand.kRight)) * (DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2.0)
    ));

    // m_storageSubsystem.setDefaultCommand(new BallCounter(m_storageSubsystem));

    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();
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
    new Button(m_controller::getBButton)
            .whenPressed(new DeployCollector(m_intakeSubsystem).andThen(new StorageCapacityChecker(m_storageSubsystem)))
            .whenReleased(new StowCollector(m_intakeSubsystem).andThen(() -> m_storageSubsystem.stop()));
    new Button(m_controller::getXButton)
            // .whenPressed(new StorageSpin(m_storageSubsystem))
              .whileHeld(new SetHoodPosition(m_shooter, 50).andThen(new Shoot(m_shooter, m_storageSubsystem)))
              .whenReleased(new BallCounterReset(m_storageSubsystem));
            // .whenPressed(new SetHoodPosition(m_shooter, 50).andThen(new SetShooterRPM(m_shooter, 1000)).andThen(new StorageSpin(m_storageSubsystem)))
            // .whenReleased(new SetShooterRPM(m_shooter, 0.0).andThen(() -> m_storageSubsystem.stop()).alongWith(new BallCounterReset(m_storageSubsystem)));
  }

  public void configureDashboard() {
    // return this.followPathCommand(true, "driveCompSwerve"); //Path not rotating, ?
    SmartDashboard.putData("drive auto", m_drivetrainSubsystem.followPathCommand(true, "driveCompSwerve")); //needs to call specific paths later.
    SmartDashboard.putData("Rotate 5", new RotateToAngle(m_drivetrainSubsystem, 5));
    SmartDashboard.putData("Rotate 45", new RotateToAngle(m_drivetrainSubsystem, 45));
    SmartDashboard.putData("Rotate 90", new RotateToAngle(m_drivetrainSubsystem, 90));
    SmartDashboard.putData("Rotate 180", new RotateToAngle(m_drivetrainSubsystem, 180));
    SmartDashboard.putData("Intake Speed 25", new IntakeSpin(m_intakeSubsystem, 0.25));
    SmartDashboard.putData("Intake Speed 50", new IntakeSpin(m_intakeSubsystem, 0.50));
    SmartDashboard.putData("Intake Speed 75", new IntakeSpin(m_intakeSubsystem, 0.75));
    SmartDashboard.putData("Intake Speed 100", new IntakeSpin(m_intakeSubsystem, 1.0));
    SmartDashboard.putData("Storage Bottom Speed 25", new StorageSpinBottom(m_storageSubsystem, 0.25));
    SmartDashboard.putData("Storage Bottom Speed 50", new StorageSpinBottom(m_storageSubsystem, 0.50));
    SmartDashboard.putData("Storage Top Speed 25", new StorageSpinTop(m_storageSubsystem, 0.25));
    SmartDashboard.putData("Storage Top Speed 50", new StorageSpinTop(m_storageSubsystem, 0.50));
    SmartDashboard.putData("Storage Run", new StorageSpin(m_storageSubsystem));
    SmartDashboard.putData("Shoot", new SetShooterRPM(m_shooter));  
    SmartDashboard.putData("Reset Ball Count", new BallCounterReset(m_storageSubsystem));
    SmartDashboard.putData("GoTo LinearServo 0 mm", new SetHoodPosition(m_shooter, 0));
    SmartDashboard.putData("GoTo LinearServo 10 mm", new SetHoodPosition(m_shooter, 10));
    SmartDashboard.putData("GoTo LinearServo 20 mm", new SetHoodPosition(m_shooter, 20));
    SmartDashboard.putData("GoTo LinearServo 30 mm", new SetHoodPosition(m_shooter, 30));
    SmartDashboard.putData("GoTo LinearServo 40 mm", new SetHoodPosition(m_shooter, 40));
    SmartDashboard.putData("GoTo LinearServo 50 mm", new SetHoodPosition(m_shooter, 50));
    SmartDashboard.putData("GoTo LinearServo 60 mm", new SetHoodPosition(m_shooter, 60));
    SmartDashboard.putData("GoTo LinearServo 70 mm", new SetHoodPosition(m_shooter, 70));
    SmartDashboard.putData("GoTo LinearServo 80 mm", new SetHoodPosition(m_shooter, 80));
    SmartDashboard.putData("GoTo LinearServo 90 mm", new SetHoodPosition(m_shooter, 90));
    SmartDashboard.putData("GoTo LinearServo 100 mm", new SetHoodPosition(m_shooter, 100));
    SmartDashboard.putData("Storage Collect Max", new StorageCapacityChecker(m_storageSubsystem));
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
    return m_drivetrainSubsystem.followPathCommand(true, "driveCompSwerve" //Path not rotating, ?

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
    // value = Math.copySign(value * value, value);

    return value;
  }
}
