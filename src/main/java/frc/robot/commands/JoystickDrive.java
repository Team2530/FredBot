/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SmartChoice;

public class JoystickDrive extends CommandBase {

  DriveTrain m_drivetrain;
  static Joystick rightStick;
  static Joystick leftStick;
  static XboxController xbox;
  /**
   * Indicies are as follows:
   * <p>
   * [0] is Stick X
   * <p>
   * [1] is Stick Y
   * <p>
   * [2] is Stick Z
   */
  static double[] leftStickValues = { 0.0, 0.0, 0.0 };
  /**
   * Indicies are as follows:
   * <p>
   * [0] is Stick X
   * <p>
   * [1] is Stick Y
   * <p>
   * [2] is Stick Z
   */
  static double[] rightStickValues = { 0.0, 0.0, 0.0 };

  SmartChoice s = new SmartChoice("Joystick Z", rightStickValues[2]);


  public JoystickDrive(DriveTrain m_drivetrain, Joystick leftStick, Joystick rightStick, XboxController xbox) {
    this.m_drivetrain = m_drivetrain;
    this.rightStick = rightStick;

    // ! LeftStick is the predominantly used stick! (In port 1)
    this.leftStick = leftStick;

    this.xbox = xbox;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateStickValues();
    if (leftStick.getRawButton(Constants.STOP_BUTTON) == false) {
      DriveTrain.arcadeDrive(Deadzone.deadZone(leftStickValues[1], 0.05), Deadzone.deadZone(leftStickValues[2], 0.05));

      s.updateValue(rightStickValues[2]);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static void updateStickValues() {
    leftStickValues[0] = leftStick.getX();
    leftStickValues[1] = leftStick.getY();
    leftStickValues[2] = leftStick.getZ();
    rightStickValues[0] = rightStick.getX();
    rightStickValues[1] = rightStick.getY();
    rightStickValues[2] = rightStick.getZ();
  }
}
