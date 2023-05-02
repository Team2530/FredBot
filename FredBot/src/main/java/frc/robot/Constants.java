// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Controller {
    public static final int JOYSTICK_PORT = 0;
    public static final int XBOX_PORT = 1;

    /** Controller Deadzones */
    public static final double DEADZONE = 0.05;
  }

  /**
   * Please use motor naming conventions when naming motors!
   * (Start from front right and travel clockwise)
   */
  public static final class Motors {
    public static final int FRONT_RIGHT_PORT = 1;
    public static final int REAR_RIGHT_PORT = 2;
    public static final int REAR_LEFT_PORT = 3;
    public static final int FRONT_LEFT_PORT = 4;

    /** Wheel Encoder Resolution */
    public static final int ENCODER_RESOLUTION = 2048;

    public static final int LEFT_ENCODER_PORT = 0;

    public static final int RIGHT_ENCODER_PORT = 2;

  }

  /** Drive Constants and Drive related simulation constants */
  public static final class Drive {
    public static final double MAX_SPEED = 1;

    /** Wheel Diameter in Meters */
    public static final double WHEEL_DIAMETER = .1524;

    /** Robot Mass in kgs (currently at 100lbs) */
    public static final double ROBOT_MASS = 45.3;

    /**
     * Distance between the robot perpendicular to the wheel's axis of rotation in
     * meters
     */
    public static final double ROBOT_TRACK_WIDTH = .3;

    /** Moment of inertia */
    public static final double MOMENT_OF_INERTIA = 2.0;

    /** DriveTrain gear ratio as output / input */
    public static final double GEAR_RATIO = 8;
  }

  /** All Autonomous realted Constants */
  public static final class Auto {

    // ----------- SysID Constants ---------- \\
    public static final double KS = 1;
    public static final double KV = 1;

    public static final double MAX_SPEED = 3.5;

    public static final double MAX_ACCELERATION = 3;

    public static final double RAMSETE_BETA = 2.0;

    public static final double RAMSETE_ZETA = 0.7;

    public static final double DRIVE_VELOCITY_CONSTANT = 5;
  }

}
