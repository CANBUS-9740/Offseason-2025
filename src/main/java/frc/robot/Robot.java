package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Utils.GameField;
import frc.robot.Utils.PathPlanner;
import frc.robot.commands.DriveStupid;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveFieldDriveCommand;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {

    private static Swerve swerveSystem;
    private XboxController xboxController;

    private PathPlanner pathPlanner;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        xboxController = new XboxController(0);
        pathPlanner = new PathPlanner();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {
        //SwerveDriveCommand command = new SwerveDriveCommand(swerveSystem, xboxController);
        swerveSystem.resetGyroToZero();
        SwerveFieldDriveCommand command = new SwerveFieldDriveCommand(swerveSystem, xboxController);
        command.schedule();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
        //new DriveStupid(swerveSystem).schedule();
        swerveSystem.resetPose(new Pose2d(new Translation2d(2.5, 6), new Rotation2d(Math.toRadians(180))));
        new SequentialCommandGroup(
                pathPlanner.goToSource(GameField.SourceStand.LEFT, GameField.SourceStandSide.CENTER),
                Commands.print("commandIsFinished")
        ).schedule();

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {
        swerveSystem.swerveDrive.resetOdometry(new Pose2d(0, 0 ,new Rotation2d(0)));
        new DriveStupid(swerveSystem).schedule();
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putNumber("modulePosition", Units.metersToInches(swerveSystem.swerveDrive.getModules()[0].getPosition().distanceMeters));
        SmartDashboard.putNumber("FLDPR", swerveSystem.swerveDrive.getModules()[0].getDriveMotor().getPosition());
    }

    @Override
    public void testExit() {

    }

    public static Swerve getSwerveSystem() {
        return swerveSystem;
    }
}
