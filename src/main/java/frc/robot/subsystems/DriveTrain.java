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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;
import frc.robot.libraries.Gains;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is Team 2530's DriveTrain class. It handles all things related to the
 * motors used to drive the robot around.
 */
public class DriveTrain extends SubsystemBase {

  /** The actual joystick input on each axis. */
  public static double[] joystickInput = { 0, 0, 0 };
  /** The current joystick interpolation on each axis. */
  public static double[] joystickLerp = { 0, 0, 0 };
  /** Last joystick input when button 3 is pressed */
  private static double[] lastJoystickInput = { 0, 0, 0 };

  private static double yawTarget = 0.0;

  // -------------------- Motors -------------------- \\
  // TODO: Specify Motors and values are all correct
  static PWMVictorSPX motorFL = new PWMVictorSPX(Constants.MOTOR_FL_DRIVE_PORT);
  static PWMVictorSPX motorFR = new PWMVictorSPX(Constants.MOTOR_FR_DRIVE_PORT);
  static PWMVictorSPX motorBL = new PWMVictorSPX(Constants.MOTOR_BL_DRIVE_PORT);
  static PWMVictorSPX motorBR = new PWMVictorSPX(Constants.MOTOR_BR_DRIVE_PORT);

  // -------------------- Joysticks --------------------- \\
  // TODO: Add/Remove Joysticks as needed
  Joystick stick;
  XboxController xbox;

  // ----------------- Shuffleboard Controls ------------------ \\
  // TODO: Put Shuffleboard controls here

  // ------------------------ PID gains ------------------------- \\
  // TODO: Set PID values

  // ------------------------ Diagnostics ------------------------- \\
  // TODO: Put Diagnostics Values Here
  AHRS ahrs;

  // ---------------------------- Other ------------------------------- \\
  static VictorSPXControlMode normalDrivingMode = VictorSPXControlMode.PercentOutput;
  static VictorSPXControlMode rampDrivingMode = VictorSPXControlMode.Velocity;

  /**
   * Creates a new {@link DriveTrain}.
   */
  public DriveTrain(AHRS ahrs, Joystick stick, XboxController xbox) {
    this.ahrs = ahrs;
    this.stick = stick;
    this.xbox = xbox;
    MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL, motorBL);
    MotorControllerGroup rightMotors = new MotorControllerGroup(motorFL, motorBL);
    DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    differentialDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    // This method will be called periodicly while the robot is enabled
  }

  public void drive(double leftSpeed, double rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
  }

  public static void stop() {
    motorFL.set(0.0);
    motorFR.set(0.0);
    motorBL.set(0.0);
    motorBR.set(0.0);
  }

  /** Stops the Robot and Resets the Yaw Angle */
  public void reset(boolean b) {
    stop();
    ahrs.reset();
  }

  /** Sets the right motors to the same speed and control mode */
  public static void setLeftSpeed(double speed) {
    motorFL.set(speed);
    motorBL.set(speed);
  }

  /** Sets the left motors to the same speed and control mode */
  public static void setRightSpeed(double speed) {
    motorFR.set(speed);
    motorBR.set(speed);
  }
}
