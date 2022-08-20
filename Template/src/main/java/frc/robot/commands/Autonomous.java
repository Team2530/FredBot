// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeedbackPanel.PanelMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.FeedbackPanel;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.kauailabs.navx.frc.AHRS;

public class Autonomous extends CommandBase {

  /** Creates a new Autonomous. */
  AHRS ahrs;
  XboxController xbox;
  DriveTrain driveTrain;
  FeedbackPanel panel;

  Timer timer = new Timer();

  public Autonomous(DriveTrain driveTrain, AHRS ahrs, XboxController xbox,
      FeedbackPanel panel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
    this.panel = panel;
    timer.start();
  }

  boolean perhaps = false;

  
  @Override
  // Auto should work if commands are placed inside the SequentialCommandGroup
  public void initialize() {
    //TODO: Specify Auto command
    SequentialCommandGroup example = new SequentialCommandGroup(
        //TODO: Insert
    );
    System.out.println("Starting Autonomous Commands...");
    System.out.println("Please don't run into something!");
    example.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO: Set a condition for when Auto is finished
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: Set a conditon for Auto to finish
    return true;
  }
}
