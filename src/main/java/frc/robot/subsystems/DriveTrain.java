/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;
import frc.robot.libraries.Gains;
import edu.wpi.first.wpilibj.XboxController;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is Team 2530's DriveTrain class. It handles all things related to the
 * motors used to drive the robot around.
 */
public class DriveTrain extends SubsystemBase {
  public static enum Cockpit {
    NONE,
    FRONT,
    LEFT,
    RIGHT
  }

  private static Cockpit cockpitMode;

  /** The actual joystick input on each axis. */
  public static double[] joystickInput = { 0, 0, 0 };
  /** The current joystick interpolation on each axis. */
  public static double[] joystickLerp = { 0, 0, 0 };
  /** Last joystick input when button 3 is pressed */
  private static double[] lastJoystickInput = { 0, 0, 0 };

  private static double yawTarget = 0.0;

  private double lastExecuted = Timer.getFPGATimestamp();

  public MecanumDrive mecanumDrive;

  // -------------------- Motors -------------------- \\
  //TODO: Specify Motors and values are all correct
  WPI_TalonFX motorFL = new WPI_TalonFX(Constants.MOTOR_FL_DRIVE_PORT);
  WPI_TalonFX motorFR = new WPI_TalonFX(Constants.MOTOR_FR_DRIVE_PORT);
  WPI_TalonFX motorBL = new WPI_TalonFX(Constants.MOTOR_BL_DRIVE_PORT);
  WPI_TalonFX motorBR = new WPI_TalonFX(Constants.MOTOR_BR_DRIVE_PORT);

  // -------------------- Joysticks --------------------- \\
  //TODO: Add/Remove Joysticks as needed
  Joystick stick;
  XboxController xbox;

  // ----------------- Shuffleboard Controls ------------------ \\
  //TODO: Put Shuffleboard controls here

  // ------------------------ PID gains ------------------------- \\
  //TODO: Set PID values 

  // ------------------------ Diagnostics ------------------------- \\
  //TODO: Put Diagnostics Values Here
  AHRS ahrs;

  // ---------------------------- Other ------------------------------- \\
  /**
   * Creates a new {@link DriveTrain}.
   */
  public DriveTrain(AHRS ahrs, Joystick stick, XboxController xbox) {
    this.ahrs = ahrs;
    this.stick = stick;
    this.xbox = xbox;
    //TODO: Specify Drivetrain Type
    // Example...
    // mecanumDrive = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
    // mecanumDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    // This method will be called periodicly while the robot is enabled
  }

  public void drive(){
    //TODO: Implement the actual driving method
  }

  public void stop() {
    //TODO: Add a stop method for the DriveTrain
  }

  public void reset(boolean b) {
    //TODO: Add a reset method for the DriveTrain
  }

}

