// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmartChoice extends SubsystemBase {

  private String title = "";
  private Double number = null;
  private String string = null;
  private Boolean bool = null;
  private Double[] numberArr = null;

  private ChangeListener listener;

  Constructor c;

  private NetworkTableEntry tableEntry;

  ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("SmartDashboard");


  private enum Constructor {
    BOOLEAN, DOUBLE, STRING, DOUBLEARRAY
  }


  public SmartChoice(String title, double number) {
    NetworkTableEntry t = Shuffleboard.getTab("SmartDashboard").add(title + "1" , number).getEntry();
    this.title = title;
    this.number = number;
    c = c.DOUBLE;
    tableEntry = shuffleboardTab.add(title, number).getEntry();
    
  }

  public SmartChoice(String title, String string) {
    this.title = title;
    this.string = string;
    c = c.STRING;
    shuffleboardTab.add(title, string);
    tableEntry = shuffleboardTab.add(title, string).getEntry();
  }

  public SmartChoice(String title, boolean bool) {
    this.title = title;
    this.bool = bool;
    c = c.BOOLEAN;
    shuffleboardTab.add(title, bool);
    tableEntry = shuffleboardTab.add(title, number).getEntry();
  }

  public SmartChoice(String title, Double[] numberArr) {
    this.title = title;
    this.numberArr = numberArr;
    c = c.DOUBLEARRAY;
    shuffleboardTab.add(title, numberArr);
    tableEntry = shuffleboardTab.add(title, number).getEntry();
  }



  @Override
  public void periodic() {}

  /**
   * An easy way to see the type of the SmartChoice created
   * @return the type of the made SmartChoice
   */
  public Constructor getType() {
    return c;
  }

  public void updateValue(double d) {
    number = d;
  }
  public void updateValue(Double[] d) {
    numberArr = d;
  }
  public void updateValue(String s) {
    string = s;
  }
  public void updateValue(Boolean b) {
    bool = b;
  }


  public void updateValue() {
    //get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    //get a reference to the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");

    //get a reference to key in "datatable" called "Y"
    NetworkTableEntry yEntry = datatable.getEntry("Y");
    inst.startClientTeam(190);

    //add an entry listener for changed values of "X", the lambda ("->" operator)
    //defines the code that should run when "X" changes
    datatable.addEntryListener("X", (table, key, entry, value, flags) -> {
       System.out.println("X changed value: " + value.getValue());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    //add an entry listener for changed values of "Y", the lambda ("->" operator)
    //defines the code that should run when "Y" changes
    yEntry.addListener(event -> {
       System.out.println("Y changed value: " + event.getEntry().getValue());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    try {
       Thread.sleep(10000);
    } catch (InterruptedException ex) {
       System.out.println("Interrupted");
       Thread.currentThread().interrupt();
       return;
    }
 }
}
