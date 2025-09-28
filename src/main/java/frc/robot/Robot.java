package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils.GameField;
import frc.robot.Utils.GroupCommands;
import frc.robot.Utils.PathPlanner;
import frc.robot.commands.DriveStupid;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ShooterSystem;

public class Robot extends TimedRobot {
    //private XboxController controller;
    //private ShooterSystem shooterSystem;

    private static Swerve swerveSystem;
    private CommandXboxController xboxController;
    private SwerveDriveCommand swerveDriveCommand;

    private PathPlanner pathPlanner;
    private GroupCommands groupCommands;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        xboxController = new CommandXboxController(0);
        pathPlanner = new PathPlanner(swerveSystem);
        groupCommands = new GroupCommands(xboxController, swerveSystem);
        //init for intake command that activates on an A button press
//        JoystickButton aButton = new JoystickButton(controller,XboxController.Button.kA.value);
//        aButton.onTrue(new IntakeCommand(shooterSystem));
//        //init for shoot command that activates on a B button press
//        JoystickButton bButton = new JoystickButton(controller,XboxController.Button.kB.value);
//        aButton.onTrue(new ShootCommand(shooterSystem));

        swerveDriveCommand = groupCommands.swerveDrive(true);

        swerveSystem.setDefaultCommand(swerveDriveCommand);

        xboxController.a().onTrue(
                new InstantCommand(()-> swerveSystem.resetPose(new Pose2d(1.5, 1.5, new Rotation2d(0))))
        );
        xboxController.b().onTrue(
                new InstantCommand(()-> swerveSystem.resetPose(new Pose2d(5.5, 2.5, new Rotation2d(0))))
        );
        xboxController.x().onTrue(
                new InstantCommand(()-> swerveSystem.resetPose(new Pose2d(3.5, 2.5, new Rotation2d(0))))
        );

        xboxController.y().onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(()-> System.out.println("startCommand")),
                        pathPlanner.goToClosestReef()
                )
        );

        xboxController.rightBumper().onTrue(
                pathPlanner.goToClosestSource()
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        groupCommands.update();
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
        swerveSystem.resetGyroToZero();
        swerveDriveCommand.setAllianceSwapper();
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
                pathPlanner.goToPoseSource(GameField.SourceStand.LEFT, GameField.SourceStandSide.CENTER),
                pathPlanner.goToPoseReef(GameField.ReefStand.STAND_1, GameField.ReefStandSide.LEFT),
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
}
