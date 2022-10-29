// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.libraries.Deadzone;
import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.schedulers.SequentialScheduler;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  AHRS ahrs;
  Timer timer = new Timer();
  DriveTrain driveTrain;
  /**
   * @param direction  1 is forward, 2 is back, 3 is right, 4 is left
   */
  public AutonomousDrive(DriveTrain driveTrain, double distance, int direction, AHRS ahrs) {
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: Set parameters for when the command gets initialized
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: Put the running code here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: Specify a condition for Auto to be done
    return true;
  }
}