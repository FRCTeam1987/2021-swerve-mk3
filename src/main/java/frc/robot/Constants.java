// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(17.5);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(17.5);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(102.39);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(61.69);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(2.64);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(309.28);

    public static final double FALCON_MAX_RPM = 6380.0;
    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final int COLLECTOR_MOTOR = 9;
    public static final int SHOOTER_MOTOR_RIGHT = 10;
    public static final int SHOOTER_MOTOR_LEFT = 11;

    public static final int STORAGE_BOTTOM_MOTOR = 15;
    public static final int STORAGE_TOP_MOTOR = 14;

    public static final int STORAGE_BALL_SENSOR = 9; // Move to Storage class
    public static final double STORAGE_BALL_TIME_DEBOUNCER = 0.1; // Move to Storage class

    public static final double DRIVETRAIN_PX_CONTROLLER = 0.25;  // TODO tune this
    public static final double DRIVETRAIN_PY_CONTROLLER = DRIVETRAIN_PX_CONTROLLER;
    public static final double DRIVETRAIN_PTHETA_CONTROLLER = 5;  // TODO tune this

    public static final class Solenoid {
        public static final int intakeSolenoid = 0;
        // public static final Solenoid
    }

    public static final class StorageSubsystem {
        public static final int startingBallCount = 0; //Change to 1 for field testing for auto
        public static final int maxBallCount = 2; 
    }
}
