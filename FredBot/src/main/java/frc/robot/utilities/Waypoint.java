package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Waypoint extends CommandBase {

    double xPos = 0.0;
    double yPos = 0.0;

    private boolean hasArrived = false;

    public Waypoint(double xPos, double yPos) {
        this.xPos = xPos;
        this.yPos = yPos;
    }

    @Override
    /**
     * Runs the Path until is within the window of error
     */
    public void execute() {
        Pose2d currentPose = DriveTrain.odometry.getPoseMeters();
        double yDiff = yPos - currentPose.getY();
        double xDiff = xPos - currentPose.getX();

        double angle = Math.toDegrees(Math.atan((yDiff) / (xDiff)));
        double speed = Math.sqrt(Math.pow(yDiff, 2) + Math.pow(xDiff, 2));

        DriveTrain.driveAtAngle(((angle + (xDiff > 0 ? 0 : 180))) % 360,
                (yDiff > 0 || xDiff > 0 ? 1 : -1) * speed);
        hasArrived = Math.abs(currentPose.getX() - xPos) < 0.2 && Math.abs(currentPose.getY() - yPos) < 0.2;

        SmartDashboard.putNumber("angle", angle);
    }

    @Override
    public boolean isFinished() {
        return hasArrived;
    }

    private int getDomainError(double yoff, double xoff) {
        if ((yoff > 0 && xoff > 0) || yoff < 0 && xoff > 0) {
            return 180;
        } else if (yoff > 0 && xoff < 0) {
            return -90;
        } else if (yoff < 0 && xoff < 0) {
            return 270;
        } else {
            return 1;
        }
    }
}
