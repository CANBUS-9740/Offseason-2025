package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;

import java.util.List;

public class PathPlanner {
    private final GameField gameField;

    public PathPlanner() {
        gameField = new GameField();
    }

    public Command goToPose(Pose2d pose) {
        System.out.println(pose.getRotation().getDegrees());
        return AutoBuilder.pathfindToPose(pose, RobotMap.PATH_CONSTRAINTS);
    }

    public Command goToPoseReef(GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide) {
        Pose2d pose = gameField.getPoseForReefStand(reefStand, reefStandSide);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                Robot.getSwerveSystem().getPose(),
                pose
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                RobotMap.PATH_CONSTRAINTS,
                null,
                new GoalEndState(0.0, pose.getRotation()));

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    public Command goToReef(GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide) {
        Pose2d pose = gameField.getPoseForReefStand(reefStand, reefStandSide);
        return goToPose(pose);
    }

    public Command goToSource(GameField.SourceStand sourceStand, GameField.SourceStandSide sourceStandSide) {
        Pose2d pose = gameField.getPoseForSource(sourceStand, sourceStandSide);
        return goToPose(pose);
    }

    public Command doSavedPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("cant run the path: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
