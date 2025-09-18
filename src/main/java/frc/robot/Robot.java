package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveStupid;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {

    private Swerve swerveSystem;
    private XboxController xboxController;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        xboxController = new XboxController(0);
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
        SwerveDriveCommand command = new SwerveDriveCommand(swerveSystem, xboxController);
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
        new DriveStupid(swerveSystem).schedule();
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
    }

    @Override
    public void testPeriodic() {
        swerveSystem.drive(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putNumber("modulePosition", Units.metersToInches(swerveSystem.swerveDrive.getModules()[0].getPosition().distanceMeters));
        SmartDashboard.putNumber("FLDPR", swerveSystem.swerveDrive.getModules()[0].getDriveMotor().getPosition());
    }

    @Override
    public void testExit() {

    }
}
