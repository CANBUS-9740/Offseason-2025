package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
    public final SwerveDrive swerveDrive;

    public Swerve() {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setCosineCompensator(true);
        swerveDrive.setAngularVelocityCompensation(false, false, 0.01);
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
        swerveDrive.setAutoCenteringModules(true);
        swerveDrive.setModuleStateOptimization(true);
        swerveDrive.synchronizeModuleEncoders();

//        swerveDrive.resetOdometry(new Pose2d(new Translation2d(2.164, 6.271), new Rotation2d(0)));

        autoBuilderConfiguration();
    }

    public void driveFieldRelative(double translationX, double translationY, double angularRotationX) {
        swerveDrive.drive(new Translation2d(
                        translationX * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS,
                        translationY * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS),
                    angularRotationX * 10,
                true,
                false);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds, new Translation2d(0 , 0));
    }

    public void stop() {
        drive(new ChassisSpeeds(0, 0, 0));
    }

    public void resetPose(Pose2d pose2d) {
        swerveDrive.resetOdometry(pose2d);
    }

    public void resetGyroToZero() {
        swerveDrive.zeroGyro();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    private void autoBuilderConfiguration() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new Error(e);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0, 0),
                        new PIDConstants(5, 0, 0)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }
}
