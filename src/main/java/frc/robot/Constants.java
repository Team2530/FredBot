/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.math.util.Units;
import frc.robot.libraries.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {
    // --------------------Motor Ports--------------------\\
    // Drivetrain motors
    // TODO: REDO PORTS FOR NEW ROBOT!
    public static final int MOTOR_FL_DRIVE_PORT = 1;
    public static final int MOTOR_FR_DRIVE_PORT = 2;
    public static final int MOTOR_BL_DRIVE_PORT = 3;
    public static final int MOTOR_BR_DRIVE_PORT = 4;

    public enum DriveMotors {
        FL, FR, BL, BR;
    }

    

    // Shooter motor port
    public static final int SHOOTER_MOTOR_PORT = 40;

    // ----------Sensor Constants-----------\\
    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;

    // ----------Driving Constants----------\\
    public static final VictorSPXControlMode normalDriveMode = VictorSPXControlMode.PercentOutput;
    public static final VictorSPXControlMode velocityDriveMode = VictorSPXControlMode.Velocity;

    public static final double brownOutVoltage = 8.00;

    // ----------Field Constants----------\\
    // The gravity on Earth (should be changed if we compete on the Moon)
    public static final double gravity = 9.81;

    // ----------Control (Joystick) Constants----------\\
    public static final double deadzone = 0.1;
    public static final double deadzoneZ = 0.4;
    public static final int stickport1 = 1; // stick port for joystick 1
    public static final int stickport2 = 2;
    public static final int xboxport = 0; // xbox controller port

    // --------------- Buttons --------------- \\

    public static final int STOP_BUTTON = 12;
}
