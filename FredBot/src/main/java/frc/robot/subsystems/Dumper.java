package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.Constants.DumperConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dumper extends SubsystemBase {

    private WPI_TalonSRX dumperMotor = new WPI_TalonSRX(DumperConstants.DUMPER_MOTOR_PORT);

    private double currentPosition = 0.0;
    private double wantedPosition = 0.0;

    public Dumper() {
        // Radians per tick
        currentPosition = dumperMotor.getSelectedSensorPosition();

    }

    @Override
    public void periodic() {
        currentPosition = dumperMotor.getSelectedSensorPosition();
        SmartDashboard.putNumber("Dumper Position (Radians)", currentPosition);
        if (currentPosition < wantedPosition) {
            dumperMotor.set(0.25);
        } else {
            dumperMotor.set(0.0);
        }
    }

    public void rotate() {
        // Rotate 60 degrees
        wantedPosition = currentPosition + Math.PI / 3;
    }

}
