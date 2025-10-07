package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils.CoralReef;
import frc.robot.Utils.GameField;
import frc.robot.Utils.GroupCommands;
import frc.robot.Utils.PathPlanner;
import frc.robot.commands.*;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Set;

import java.util.Optional;

public class Robot extends TimedRobot {
    //private XboxController controller;
    //private ShooterSystem shooterSystem;

    private static Swerve swerveSystem;
    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;

    private PathPlanner pathPlanner;
    private GroupCommands groupCommands;
    private Limelight limelight;

    private EventLoop gameLoop;
    private EventLoop testLoop;

    @Override
    public void robotInit() {
        gameLoop = new EventLoop();
        testLoop = new EventLoop();
        CommandScheduler.getInstance().setActiveButtonLoop(gameLoop);
        SmartDashboard.putString("activeButtonLoop: ", "gameLoop");

        swerveSystem = new Swerve();
        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);
        pathPlanner = new PathPlanner(swerveSystem);
        groupCommands = new GroupCommands(driverController, swerveSystem);
        limelight = new Limelight("limelight-edi");
        //init for intake command that activates on an A button press
//        JoystickButton aButton = new JoystickButton(controller,XboxController.Button.kA.value);
//        aButton.onTrue(new IntakeCommand(shooterSystem));
//        //init for shoot command that activates on a B button press
//        JoystickButton bButton = new JoystickButton(controller,XboxController.Button.kB.value);
//        aButton.onTrue(new ShootCommand(shooterSystem));

        operationController.start(gameLoop).onTrue(Commands.runOnce(()-> CommandScheduler.getInstance().setActiveButtonLoop(testLoop)));
        operationController.start(testLoop).onTrue(Commands.runOnce(()-> CommandScheduler.getInstance().setActiveButtonLoop(gameLoop)));

        swerveDriveCommand = groupCommands.swerveDrive(true);

        groupCommands.elevatorSystem.setDefaultCommand(groupCommands.elevatorMoveCommand);

        swerveSystem.setDefaultCommand(swerveDriveCommand);
//
//        xboxController.a().onTrue(
//                new InstantCommand(()-> swerveSystem.resetPose(new Pose2d(1.5, 1.5, new Rotation2d(0))))
//        );
//        xboxController.b().onTrue(
//                new InstantCommand(()-> swerveSystem.resetPose(new Pose2d(5.5, 2.5, new Rotation2d(0))))
//        );
//        xboxController.x().onTrue(
//                new InstantCommand(()-> swerveSystem.resetPose(new Pose2d(3.5, 2.5, new Rotation2d(0))))
//        );


        //driver regular teleop
        driverController.a(gameLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L1_HEIGHT_M)
        );

        driverController.b(gameLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L2_HEIGHT_M)
        );

        driverController.y(gameLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L3_HEIGHT_M)
        );

        driverController.x(gameLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L4_HEIGHT_M)
        );

        driverController.rightBumper(gameLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_PARKING_HEIGHT_M)
        );

        driverController.a(testLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L1_HEIGHT_M)
        );

        driverController.b(testLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L2_HEIGHT_M)
        );

        driverController.y(testLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L3_HEIGHT_M)
        );

        driverController.x(testLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_L4_HEIGHT_M)
        );

        driverController.rightBumper(testLoop).onTrue(
                groupCommands.goToHeightCommand(RobotMap.ELEVATOR_PARKING_HEIGHT_M)
        );

        driverController.pov(0, 0, gameLoop).onTrue(groupCommands.shootCommand(RobotMap.SHOOTER_MOTOR_OUTTAKE_DEFAULT_SPEED));
        driverController.pov(0, 180, gameLoop).onTrue(groupCommands.intakeCommand());

        driverController.pov(0, 0, testLoop).onTrue(groupCommands.shootCommand(RobotMap.SHOOTER_MOTOR_OUTTAKE_DEFAULT_SPEED));
        driverController.pov(0, 180, testLoop).onTrue(groupCommands.intakeCommand());


        driverController.pov(0, 270, gameLoop).onTrue(groupCommands.shootCommand(RobotMap.SHOOTER_MOTOR_OUTTAKE_LOWER_SPEED));
        driverController.pov(0, 270, testLoop).onTrue(groupCommands.shootCommand(RobotMap.SHOOTER_MOTOR_OUTTAKE_LOWER_SPEED));

        //operator pre target teleop

        operationController.a(gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageRight(CoralReef.PODIUM)
        );

        operationController.b(gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageRight(CoralReef.FIRST_STAGE)
        );

        operationController.x(gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageRight(CoralReef.SECOND_STAGE)
        );

        operationController.y(gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageRight(CoralReef.THIRD_STAGE)
        );

        operationController.pov(0, 180, gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageLeft(CoralReef.PODIUM)
        );

        operationController.pov(0, 90, gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageLeft(CoralReef.FIRST_STAGE)
        );

        operationController.pov(0, 0, gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageLeft(CoralReef.SECOND_STAGE)
        );

        operationController.pov(0, 270, gameLoop).onTrue(
                groupCommands.coralOnClosestReefStageLeft(CoralReef.THIRD_STAGE)
        );

       operationController.rightBumper(gameLoop).onTrue(
               groupCommands.getCoralFromClosestSource(GameField.SourceStandSide.CENTER)
       );

        operationController.leftBumper(gameLoop).onTrue(
                groupCommands.getCoralFromClosestSource(GameField.SourceStandSide.LEFT)
        );


        //operator test heights
        operationController.a(testLoop).onTrue(
                Commands.defer(() -> {
                    double currentHeight = groupCommands.elevatorSystem.getHeightMeters();
                    double newHeight = currentHeight + 0.05;

                    if (newHeight > RobotMap.ELEVATOR_MAX_HEIGHT_M) {
                        return Commands.none();
                    }

                    return groupCommands.goToHeightCommand(newHeight);
                }, Set.of())
        );

        operationController.x(testLoop).onTrue(
                Commands.defer(() -> {
                    double currentHeight = groupCommands.elevatorSystem.getHeightMeters();
                    double newHeight = currentHeight + 0.01;

                    if (newHeight > RobotMap.ELEVATOR_MAX_HEIGHT_M) {
                        return Commands.none();
                    }

                    return groupCommands.goToHeightCommand(newHeight);
                }, Set.of())
        );

        operationController.b(testLoop).onTrue(
                Commands.defer(() -> {
                    double currentHeight = groupCommands.elevatorSystem.getHeightMeters();
                    double newHeight = currentHeight - 0.05;

                    if (newHeight < RobotMap.ELEVATOR_MIN_HEIGHT_M) {
                        return Commands.none();
                    }

                    return groupCommands.goToHeightCommand(newHeight);
                }, Set.of())
        );

        operationController.y(testLoop).onTrue(
                Commands.defer(() -> {
                    double currentHeight = groupCommands.elevatorSystem.getHeightMeters();
                    double newHeight = currentHeight - 0.01;

                    if (newHeight < RobotMap.ELEVATOR_MIN_HEIGHT_M) {
                        return Commands.none();
                    }

                    return groupCommands.goToHeightCommand(newHeight);
                }, Set.of())
        );


        //driver reset commands

        driverController.leftBumper(testLoop).onTrue(groupCommands.resetCommand());
        driverController.leftBumper(gameLoop).onTrue(groupCommands.resetCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        groupCommands.update();
        if (CommandScheduler.getInstance().getActiveButtonLoop() == gameLoop) {
            SmartDashboard.putString("activeButtonLoop: ", "gameLoop");
        } else {
            SmartDashboard.putString("activeButtonLoop: ", "testLoop");
        }

        Optional<LimelightHelpers.PoseEstimate> poseEstimateOptional = limelight.getPose();
        if(poseEstimateOptional.isPresent()){
            LimelightHelpers.PoseEstimate poseEstimate = poseEstimateOptional.get();
            swerveSystem.addVisionMeasurement(poseEstimate);
        }
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
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {
    }
}
