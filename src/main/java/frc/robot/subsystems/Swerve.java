package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameField;
import frc.robot.RobotMap;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Swerve extends SubsystemBase {
    public SwerveDrive swerveDrive;
    private static GameField gameField;
    private GameField.SelectedSourceStand sourceStand;

    public Swerve() {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        gameField = new GameField();
        sourceStand = new GameField.SelectedSourceStand(GameField.SourceStand.RIGHT, GameField.SourceStandSide.CENTER, swerveDrive.getPose());

        swerveDrive = new SwerveDrive(swerveDrive.swerveDriveConfiguration, swerveDrive.swerveController.config, RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS,swerveDrive.getPose());

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
        swerveDrive.synchronizeModuleEncoders();
    }

    public void driveFieldRelative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS,
                        translationY.getAsDouble() * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS),
                angularRotationX.getAsDouble() * 10,
                true,
                false);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public void stop() {

    }

    public void resetPose(Pose2d pose2d) {
        swerveDrive.resetOdometry(pose2d);
    }

    private void pathPlannerSetUp() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new Error(e);
        }

        AutoBuilder.configure(
                swerveDrive::getPose, // Robot pose supplier
                swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speedsRobotRelative, moduleFeedForwards) -> {
                    drive(speedsRobotRelative);
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            swerveDrive.field.getObject("Trajectory").setPoses(poses);
        });
    }

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(new Translation2d(sourceStand.pose.getX(), sourceStand.pose.getY()), new Rotation2d(sourceStand.pose.getRotation().getRadians()))
    );

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(180))
    );


    public Command followPath(PathPlannerPath path){
        try{
            return AutoBuilder.followPath(path);

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }


        }

}
