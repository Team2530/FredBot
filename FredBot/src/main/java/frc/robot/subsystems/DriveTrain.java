// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.utilities.Cycle;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  // ---------- Motor Output Percents ---------- \\
  public static double xPercent = 0.0;
  public static double yPercent = 0.0;
  public static double zPercent = 0.0;

  public static enum ControlMode {
    Driver,
    Auto,
    Vision,
    Stop
  }

  private static ControlMode driveControlMode = ControlMode.Stop;

  /** NavX gyroscope */
  public static final AHRS navX = new AHRS();

  // ---------- Motors & Encoders ---------- \\
  public final WPI_VictorSPX FRONT_RIGHT_MOTOR = new WPI_VictorSPX(Motors.FRONT_RIGHT_PORT);
  public final WPI_VictorSPX REAR_RIGHT_MOTOR = new WPI_VictorSPX(Motors.REAR_RIGHT_PORT);
  public final WPI_VictorSPX FRONT_LEFT_MOTOR = new WPI_VictorSPX(Motors.FRONT_LEFT_PORT);
  public final WPI_VictorSPX REAR_LEFT_MOTOR = new WPI_VictorSPX(Motors.REAR_LEFT_PORT);

  private MotorControllerGroup rightMotors = new MotorControllerGroup(FRONT_RIGHT_MOTOR, REAR_RIGHT_MOTOR);
  private MotorControllerGroup leftMotors = new MotorControllerGroup(FRONT_LEFT_MOTOR, REAR_LEFT_MOTOR);

  private final static Encoder leftEncoder = new Encoder(Motors.LEFT_ENCODER_PORT, Motors.LEFT_ENCODER_PORT + 1);
  private final static Encoder rightEncoder = new Encoder(Motors.RIGHT_ENCODER_PORT, Motors.RIGHT_ENCODER_PORT + 1);

  public static final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navX.getRotation2d(),
      leftEncoder.getDistance(), rightEncoder.getDistance());

  public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Drive.ROBOT_TRACK_WIDTH);

  public static DifferentialDrive driveOutputs;

  // ---------- DriveTrain Simulation Helpers ---------- \\

  private EncoderSim simLeftEncoder = new EncoderSim(leftEncoder);
  private EncoderSim simRightEncoder = new EncoderSim(rightEncoder);

  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId
      .createDrivetrainVelocitySystem(DCMotor.getCIM(2), Drive.ROBOT_MASS, Drive.WHEEL_DIAMETER / 2,
          Drive.ROBOT_TRACK_WIDTH / 2, Drive.MOMENT_OF_INERTIA, Drive.GEAR_RATIO);

  private final DifferentialDrivetrainSim driveTrainSimulator = new DifferentialDrivetrainSim(
      m_drivetrainSystem, DCMotor.getCIM(2), Drive.GEAR_RATIO, Drive.ROBOT_TRACK_WIDTH,
      Drive.WHEEL_DIAMETER / 2, null);

  // ---------- PID Controllers ---------- \\

  private static PIDController rotationController = new PIDController(0.023, 0.02, 0.005);

  // ---------- Path Following ---------- \\

  public static final DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Auto.KS, Auto.KV), kinematics, 10);

  public static final TrajectoryConfig config = new TrajectoryConfig(Auto.MAX_SPEED, Auto.MAX_ACCELERATION)
      .setKinematics(kinematics).addConstraint(voltageConstraint);

  // ---------- Other ---------- \\
  /** Field Simulation */
  private final Field2d field = new Field2d();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Invert the left side motors as they will be backwards
    leftMotors.setInverted(true);

    // Create a drivetrain
    driveOutputs = new DifferentialDrive(leftMotors, rightMotors);

    // Set Distance per pulse on encoders
    leftEncoder.setDistancePerPulse(Math.PI * Drive.WHEEL_DIAMETER / Motors.ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(Math.PI * Drive.WHEEL_DIAMETER / Motors.ENCODER_RESOLUTION);

    // Put odometry to field widget
    SmartDashboard.putData(field);

  }

  @Override
  public void periodic() {
    updateOdometry();
    field.setRobotPose(odometry.getPoseMeters());

    // ? Change Drive Style based on current drive mode
    switch (driveControlMode) {
      case Auto:
        break;
      case Driver:
        driverControl();
        break;
      case Stop:
        driveOutputs.arcadeDrive(0.0, 0.0);
        break;
      case Vision:
        break;
    }
  }

  /** Updates the current Odometry based on the motor encoders */
  private void updateOdometry() {
    // update based on actual robot running or on simulation
    if (Robot.isReal()) {
      odometry.update(navX.getRotation2d(), leftEncoder.getDistance(),
          rightEncoder.getDistance());
    } else {
      driveTrainSimulator.setInputs(
          -FRONT_LEFT_MOTOR.get() * RobotController.getInputVoltage(),
          FRONT_RIGHT_MOTOR.get() * RobotController.getInputVoltage());
      driveTrainSimulator.update(0.02);

      simLeftEncoder.setDistance(driveTrainSimulator.getLeftPositionMeters());
      simLeftEncoder.setRate(driveTrainSimulator.getLeftVelocityMetersPerSecond());
      simRightEncoder.setDistance(driveTrainSimulator.getRightPositionMeters());
      simRightEncoder.setRate(driveTrainSimulator.getRightVelocityMetersPerSecond());

      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,
          "Yaw"));
      angle.set(-driveTrainSimulator.getHeading().getDegrees());

      // Update actual odometry based on calculated values
      odometry.update(navX.getRotation2d(), simLeftEncoder.getDistance(), simRightEncoder.getDistance());
    }
  }

  public static void resetOdometry(Pose2d pose) {
    odometry.resetPosition(navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }

  /**
   * Runs the Robot in DriverControl Mode
   */
  private void driverControl() {
    updateStickValues();
    if (Robot.isReal()) {
      driveOutputs.arcadeDrive(-yPercent, zPercent);
    } else {
      // Robot turns quickly, so we tone it down in simulation
      driveOutputs.arcadeDrive(-yPercent, zPercent * 0.4);
    }

  }

  private void updateStickValues() {
    xPercent = RobotContainer.JOYSTICK.getX();
    yPercent = RobotContainer.JOYSTICK.getY();
    zPercent = RobotContainer.JOYSTICK.getZ();

  }

  public static void changeDriveMode(ControlMode c) {
    driveControlMode = c;
  }

  /**
   * Points to the specified angle
   * 
   * @param angle angle to point towards
   */
  public static void driveAtAngle(double angle, double speed) {
    zPercent = -rotationController.calculate(navX.getAngle() % 360, -angle);
    xPercent = speed * Math.cos(Math.toRadians(Math.abs(navX.getAngle() % 360) - angle));

    driveOutputs.arcadeDrive(xPercent, zPercent);
  }

  public static Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    driveOutputs.feed();
  }

}
