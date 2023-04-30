package frc.robot.utilities;

import java.rmi.server.ExportException;
import java.util.ArrayList;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Path extends CommandBase {

    SequentialCommandGroup executePath = new SequentialCommandGroup();

    public Path(Waypoint... waypoints) {
        for (Waypoint w : waypoints) {
            executePath.addCommands(w);
        }
    }

    @Override
    public void schedule() {
        executePath.schedule();
    }

    @Override
    public boolean isFinished() {
        return executePath.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        executePath.cancel();
    }

}
