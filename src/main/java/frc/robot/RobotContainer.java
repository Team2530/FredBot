/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.FeedbackPanel.PanelMode;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // -------------------- Joysticks and Buttons -------------------- \\
  // Joysticks
  final Joystick stick1 = new Joystick(Constants.stickport1); // Creates a joystick on port 1

  // Xbox Controller
  final XboxController xbox = new XboxController(Constants.xboxport);

  // The robot's subsystems and commands are defined here...

  // -------------------- Subsystems -------------------- \\

  // Inputs
  private final AHRS m_ahrs = new AHRS();
  private final USBCamera usbCamera = new USBCamera();

  // Outputs
  // TODO: Specify new values to pass into DriveTrain such as the controller values
  public final DriveTrain m_driveTrain = new DriveTrain(m_ahrs, stick1, xbox);
  private final Indicators lights = new Indicators(3);
  // private final PhotonVision vision = new PhotonVision(stick1, m_driveTrain, lights);


  // -------------------- Autonomous Commands -------------------- \\
  // insert autonomous commands here

  // -------------------- Telop Commands -------------------- \\
  // insert teleop commands here

  // -------------------- Global Toggles -------------------- \\
  // Place Toggles usually disabled by button presse here (Things such as 
  // automatic gathering/scoring, speed mode, etc.)

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  /**
   * This method sets the button mapping and allows buttons to 
   * make thinsg happen
   */
  private void configureButtonBindings() {}

  

  /**
   * Use this to pass the enable command to the main {@link Robot} class.
   * This command is run immediately when the robot is enabled (not simply turned
   * on), regardless of whether the robot is in teleop or autonomous.
   *
   * @return the command to run when the robot is enabled
   */
  public Command getEnableCommand() {
    return new InstantCommand(() -> m_driveTrain.reset(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Create an Autonomous command to return here
    return null;
  }

  public Command getTelopCommand() {
    // TODO: Create a new Telop command to return here
    return null;
  }

  public Command getTestCommand() {
    // TODO: Create a new Test command to return here
    return null;
  }
}
