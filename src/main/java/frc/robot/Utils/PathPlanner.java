package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;

import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;

public class PathPlanner {
    private final GameField gameField;
    private Boolean isInAutoMovement = false;
    private final Swerve swerve;

    public PathPlanner(Swerve swerve) {
        gameField = new GameField();
        this.swerve = swerve;
    }

    public Command goToPosePathFind(Pose2d pose) {
        return new SequentialCommandGroup(
                Commands.runOnce(()-> isInAutoMovement = true),
                AutoBuilder.pathfindToPose(pose, RobotMap.PATH_CONSTRAINTS),
                Commands.runOnce(()-> isInAutoMovement = false)
        );
    }

    public Command goToPose(Pose2d pose) {
        return Commands.defer(()-> {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    swerve.getPose(),
                    pose
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    RobotMap.PATH_CONSTRAINTS,
                    null,
                    new GoalEndState(0.0, pose.getRotation()));

            path.preventFlipping = true;

            return new SequentialCommandGroup(
                    Commands.runOnce(()-> {
                        isInAutoMovement = true;
                        swerve.getField().getObject("Target").setPose(pose);
                    }),
                    AutoBuilder.followPath(path),
                    Commands.runOnce(()-> isInAutoMovement = false)
            );
        }, Set.of(swerve));
    }

    public Command goToPoseReef(GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide) {
        Pose2d pose = gameField.getPoseForReefStand(reefStand, reefStandSide);
        return goToPose(pose);
    }

    public Command goToPreTargetReefPose(GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide) {
        return Commands.defer(()-> {
            Pose2d reefPose = gameField.getPoseForReefStand(reefStand, reefStandSide);
            Pose2d preReefPose = gameField.getPreTargetPose(reefPose);

            return goToPose(preReefPose);
        }, Set.of(swerve));
    }

    public Command goToPreTargetClosestReefPose(GameField.ReefStandSide reefStandSide) {
        return Commands.defer(()-> {
            Optional<GameField.SelectedReefStand> closestReef = gameField.findBestReefStandTo(swerve.getPose(), reefStandSide, false);

            if (closestReef.isEmpty()) {
                return Commands.none();
            }

            Pose2d reefPose = gameField.getPoseForReefStand(closestReef.get().stand, reefStandSide);
            Pose2d preReefPose = gameField.getPreTargetPose(reefPose);

            return goToPose(preReefPose);
        }, Set.of(swerve));
    }

    public Command goToPoseSource(GameField.SourceStand sourceStand, GameField.SourceStandSide sourceStandSide) {
        Pose2d pose = gameField.getPoseForSource(sourceStand, sourceStandSide);
        return goToPose(pose);
    }

    public Command goToReefPathFind(GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide) {
        Pose2d pose = gameField.getPoseForReefStand(reefStand, reefStandSide);
        return goToPose(pose);
    }

    public Command goToSourcePathFind(GameField.SourceStand sourceStand, GameField.SourceStandSide sourceStandSide) {
        Pose2d pose = gameField.getPoseForSource(sourceStand, sourceStandSide);
        return goToPose(pose);
    }

    public Command goToClosestSource(GameField.SourceStandSide sourceStandSide) {
        return Commands.defer(() -> {
            Optional<GameField.SelectedSourceStand> closestSource = gameField.getClosestSourceTo(swerve.getPose(), sourceStandSide);

            if (closestSource.isEmpty()) {
                System.out.println("bla bla");
                return Commands.none();
            }

            GameField.SelectedSourceStand source = closestSource.get();
            System.out.println("fuck this");
            return goToPoseSource(source.stand, source.side);
        }, Set.of(swerve));
    }

    public Command goToClosestReef(GameField.ReefStandSide reefStandSide) {
        return Commands.defer(() -> {
            Optional<GameField.SelectedReefStand> closestReef = gameField.findBestReefStandTo(swerve.getPose(), reefStandSide,false);

            if (closestReef.isEmpty()) {
                return Commands.none();
            }

            GameField.SelectedReefStand reef = closestReef.get();
            reef.side = reefStandSide;
            return goToPoseReef(reef.stand, reef.side);
        }, Set.of(swerve));
    }

    public Command followPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("cant run the path: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public boolean closestSourceIsPresent(GameField.SourceStandSide sourceStandSide) {
        return gameField.getClosestSourceTo(swerve.getPose(), sourceStandSide).isPresent();
    }

    public boolean closestReefIsPresent(GameField.ReefStandSide reefStandSide) {
        return gameField.findBestReefStandTo(swerve.getPose(), reefStandSide, false).isPresent();
    }

    public void update() {
        Optional<GameField.SelectedSourceStand> closestSource = gameField.getClosestSourceTo(swerve.getPose(), GameField.SourceStandSide.CENTER);
        if (closestSource.isPresent()) {
            GameField.SelectedSourceStand stand = closestSource.get();
            SmartDashboard.putString("ClosestSource", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name(), stand.side.name()));
            swerve.getField().getObject("ClosestSource").setPose(stand.pose);
        } else {
            SmartDashboard.putString("ClosestSource", "");
            swerve.getField().getObject("ClosestSource").setPoses();
        }

        Optional<GameField.SelectedReefStand> closestReefLeft = gameField.findBestReefStandTo(swerve.getPose(), GameField.ReefStandSide.LEFT,false);
        if (closestReefLeft.isPresent()) {
            GameField.SelectedReefStand stand = closestReefLeft.get();
            swerve.getField().getObject("ClosestStandLeft").setPose(stand.pose);
            SmartDashboard.putString("ClosestStandLeft", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name(), stand.side.name()));

            Pose2d reefPose = gameField.getPoseForReefStand(stand.stand, stand.side);
            Pose2d preTargetPose = gameField.getPreTargetPose(reefPose);
            swerve.getField().getObject("ClosestPreTargetReefPoseLeft").setPose(preTargetPose);
            SmartDashboard.putString("ClosestPreTargetReefPoseLeft", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name() + "_PreTarget", stand.side.name() + "_PreTarget"));
        } else {
            swerve.getField().getObject("ClosestStandLeft").setPoses();
            SmartDashboard.putString("ClosestStandLeft", "");

            swerve.getField().getObject("ClosestPreTargetReefPoseLeft").setPoses();
            SmartDashboard.putString("ClosestPreTargetReefPoseLeft", "");
        }

        Optional<GameField.SelectedReefStand> closestReefRight = gameField.findBestReefStandTo(swerve.getPose(), GameField.ReefStandSide.RIGHT,false);
        if (closestReefRight.isPresent()) {
            GameField.SelectedReefStand stand = closestReefRight.get();
            swerve.getField().getObject("ClosestStandRight").setPose(stand.pose);
            SmartDashboard.putString("ClosestStandRight", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name(), stand.side.name()));

            Pose2d reefPose = gameField.getPoseForReefStand(stand.stand, stand.side);
            Pose2d preTargetPose = gameField.getPreTargetPose(reefPose);
            swerve.getField().getObject("ClosestPreTargetReefPoseRight").setPose(preTargetPose);
            SmartDashboard.putString("ClosestPreTargetReefPoseRight", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name() + "_PreTarget", stand.side.name() + "_PreTarget"));
        } else {
            swerve.getField().getObject("ClosestStandRight").setPoses();
            SmartDashboard.putString("ClosestStandRight", "");

            swerve.getField().getObject("ClosestPreTargetReefPoseRight").setPoses();
            SmartDashboard.putString("ClosestPreTargetReefPoseRight", "");
        }

        SmartDashboard.putBoolean("isInsAutoMovement", isInAutoMovement);
    }
}
