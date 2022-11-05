// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.ScheduledFuture;
import java.util.function.DoubleSupplier;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmartChoice extends SubsystemBase {

  private NetworkTableEntry tableEntry;
  private String title;

  public SmartChoice(String title, NetworkTableEntry tableEntry) {
    this.tableEntry = tableEntry;
    this.title = title;
    Shuffleboard.getTab("SmartDashboard").add(title, tableEntry);
  }

  @Override
  public void periodic() {
    // update(title, tableEntry);
  }

  private void update(String title, NetworkTableEntry tableEntry) {
    Shuffleboard.getTab("SmartDashboard").add(title, tableEntry);

  }

  public static NetworkTableEntry makeEntry(String title, double number) {
    return Shuffleboard.getTab("Shuffleboard").add(title, number).getEntry();
  }

  public static NetworkTableEntry makeEntry(String title, String str) {
    return Shuffleboard.getTab("Shuffleboard").add(title, str).getEntry();
  }

  public static NetworkTableEntry makeEntry(String title, boolean bool) {
    return Shuffleboard.getTab("Shuffleboard").add(title, bool).getEntry();
  }

  public static NetworkTableEntry makeEntry(String title, double[] nums) {
    return Shuffleboard.getTab("Shuffleboard").add(title, nums).getEntry();
  }
}
