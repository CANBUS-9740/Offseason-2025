package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    public Swerve() {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        final double maximumSpeed = 5;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
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
}
