package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auto;
import frc.robot.subsystems.DriveTrain;

/** Holds path following command used for a cycle */
public class Cycle extends CommandBase {

    RamseteCommand pathCommand;
    DriveTrain driveTrain = RobotContainer.DRIVE_TRAIN;

    public Cycle(Trajectory trajectory) {
        pathCommand = new RamseteCommand(trajectory, DriveTrain::getPose,
                new RamseteController(Auto.RAMSETE_BETA, Auto.RAMSETE_ZETA),
                new SimpleMotorFeedforward(Auto.KS, Auto.KV), DriveTrain.kinematics, DriveTrain::getWheelSpeeds,
                new PIDController(Auto.DRIVE_VELOCITY_CONSTANT, 0, 0),
                new PIDController(Auto.DRIVE_VELOCITY_CONSTANT, 0, 0), driveTrain::tankDriveVolts,
                driveTrain);
        // Stops Robot when finished
        pathCommand.andThen(() -> RobotContainer.DRIVE_TRAIN.tankDriveVolts(0, 0));
    }

    @Override
    public void schedule() {
        pathCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
